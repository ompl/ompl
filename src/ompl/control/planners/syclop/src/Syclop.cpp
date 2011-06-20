#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/base/GoalState.h"
#include "ompl/base/ProblemDefinition.h"

ompl::control::Syclop::Syclop(const SpaceInformationPtr &si, Decomposition &d) : ompl::base::Planner(si, "Syclop"),
    siC_(si.get()), decomp(d), graph(decomp.getNumRegions()), covGrid(COVGRID_LENGTH, 2, d)
{
}

ompl::control::Syclop::~Syclop()
{
}

void ompl::control::Syclop::setup(void)
{
    base::Planner::setup();
    buildGraph();
    const base::ProblemDefinitionPtr& pdef = getProblemDefinition();
    base::State *start = pdef->getStartState(0);
    /* Here we are temporarily assuming that the goal is of type GoalState.
     * This is to find a Region corresponding to the goal. */
    base::State *goal = pdef->getGoal()->as<base::GoalState>()->state;
    startRegion = decomp.locateRegion(start);
    goalRegion = decomp.locateRegion(goal);
    Motion* startMotion = initializeTree(start);
    graph[boost::vertex(startRegion,graph)].motions.push_back(startMotion);
    updateCoverageEstimate(graph[boost::vertex(startRegion,graph)], start);

    std::cout << "start is " << startRegion << std::endl;
    std::cout << "goal is " << goalRegion << std::endl;
    setupRegionEstimates();
    updateRegionEstimates();
    updateEdgeEstimates();
    printRegions();
    printEdges();
}

void ompl::control::Syclop::clear(void)
{
    base::Planner::clear();
    regionsToEdge.clear();
    lead.clear();
    avail.clear();
    availDist.clear();
}

bool ompl::control::Syclop::solve(const base::PlannerTerminationCondition &ptc)
{
    std::set<Motion*> newMotions;
    base::Goal* goal = getProblemDefinition()->getGoal().get();
    while (!ptc())
    {
        std::cout << "computing new lead" << std::endl;
        computeLead();
        computeAvailableRegions();
        for (int i = 0; i < NUM_AVAIL_EXPLORATIONS; ++i)
        {
            std::cout << "exploration " << i+1 << " of " << NUM_AVAIL_EXPLORATIONS << std::endl;
            const int region = selectRegion();
            bool improved = false;
            for (int j = 0; j < NUM_TREE_SELECTIONS; ++j)
            {
                std::cout << "tree selection " << j+1 << " of " << NUM_TREE_SELECTIONS << std::endl;
                newMotions.clear();
                selectAndExtend(graph[boost::vertex(region,graph)], newMotions);
                if (!newMotions.empty())
                    std::cout << "planner returned " << newMotions.size() << " new motions" << std::endl;
                for (std::set<Motion*>::const_iterator m = newMotions.begin(); m != newMotions.end(); ++m)
                {
                    Motion* motion = *m;
                    base::State* state = motion->state;
                    if (goal->isSatisfied(state))
                    {
                        std::vector<const Motion*> mpath;
                        const Motion* solution = motion;
                        while (solution != NULL)
                        {
                            mpath.push_back(solution);
                            solution = solution->parent;
                        }
                        PathControl* path = new PathControl(si_);
                        for (int i = mpath.size()-1; i >= 0; --i)
                        {
                            path->states.push_back(si_->cloneState(mpath[i]->state));
                            if (mpath[i]->parent)
                            {
                                path->controls.push_back(siC_->cloneControl(mpath[i]->control));
                                path->controlDurations.push_back(mpath[i]->steps * siC_->getPropagationStepSize());
                            }
                        }
                        //TODO include approximate solution
                        goal->setSolutionPath(base::PathPtr(path), false);
                        return true;
                    }
                    const int oldRegion = decomp.locateRegion(motion->parent->state);
                    const int newRegion = decomp.locateRegion(state);
                    std::cout << "created motion from region " << oldRegion << " to region " << newRegion << std::endl;
                    graph[boost::vertex(newRegion,graph)].motions.push_back(motion);
                    if (newRegion != oldRegion)
                    {
                        avail.insert(newRegion);
                        Adjacency& adj = regionsToEdge.find(std::pair<int,int>(oldRegion,newRegion))->second;
                        adj.empty = false;
                        ++adj.numSelections;
                        improved |= updateConnectionEstimate(graph[boost::vertex(oldRegion,graph)], graph[boost::vertex(newRegion,graph)], state);
                    }
                    improved |= updateCoverageEstimate(graph[boost::vertex(newRegion, graph)], state);
                    if (!improved)
                        std::cout << "no improvement to connections or coverage, yet" << std::endl;
                    updateRegionEstimates();
                    updateEdgeEstimates();
                    std::cout << "updated estimates" << std::endl;
                    printRegions();
                    printEdges();
                }
            }
            if (!improved && rng.uniform01() < PROB_ABANDON_LEAD_EARLY)
            {
                std::cout << "abandoning lead early" << std::endl;
                break;
            }
        }
    }
    return false;
}

bool ompl::control::Syclop::solve(double solveTime)
{
    return ompl::base::Planner::solve(solveTime);
}

void ompl::control::Syclop::printRegions(void)
{
    for (int i = 0; i < decomp.getNumRegions(); ++i)
    {
        Region& r = graph[boost::vertex(i, graph)];
        std::cout << "Region " << r.index << ": ";
        std::cout << "numMotions=" << r.motions.size() << ",";
        std::cout << "nselects=" << r.numSelections << ",";
        std::cout << "vol=" << r.volume << ",";
        std::cout << "freeVol=" << r.freeVolume << ",";
        std::cout << "pcentValid=" << r.percentValidCells << ",";
        std::cout << "numCells=" << r.covGridCells.size() << ",";
        std::cout << "weight=" << r.weight << ",";
        std::cout << "alpha=" << r.alpha << "";
        std::cout << std::endl;
    }
}

void ompl::control::Syclop::printEdges(void)
{
    EdgeIter ei, end;
    VertexIndexMap index = get(boost::vertex_index, graph);
    for (boost::tie(ei,end) = boost::edges(graph); ei != end; ++ei)
    {
        const Adjacency& a = graph[*ei];
        std::cout << "Edge (" << index[boost::source(*ei, graph)] << "," << index[boost::target(*ei, graph)] << "): ";
        std::cout << "numCells=" << a.covGridCells.size() << ",";
        std::cout << "nselects=" << a.numSelections << ",";
        std::cout << "cost=" << a.cost << std::endl;
    }
}

void ompl::control::Syclop::initEdge(Adjacency& adj, Region* r, Region* s)
{
    adj.empty = true;
    adj.numLeadInclusions = 0;
    adj.numSelections = 0;
    std::pair<int,int> regions(r->index, s->index);
    std::pair<std::pair<int,int>,Adjacency&> mapping(regions, adj);
    regionsToEdge.insert(mapping);
}

void ompl::control::Syclop::updateEdgeEstimates(void)
{
    EdgeIter ei, end;
    VertexIndexMap index = get(boost::vertex_index, graph);
    for (boost::tie(ei,end) = boost::edges(graph); ei != end; ++ei)
    {
        Adjacency& a = graph[*ei];
        const double nsel = (a.empty ? a.numLeadInclusions : a.numSelections);
        a.cost = (1 + nsel*nsel) / (1 + a.covGridCells.size());
        const Region& source = graph[boost::source(*ei,graph)];
        const Region& target = graph[boost::target(*ei,graph)];
        a.cost *= source.alpha * target.alpha;
    }
}

void ompl::control::Syclop::initRegion(Region& r)
{
    r.numSelections = 0;
    r.volume = 1.0;
    r.percentValidCells = 1.0;
    r.freeVolume = 1.0;
}

void ompl::control::Syclop::setupRegionEstimates(void)
{
    std::vector<int> numTotal(decomp.getNumRegions(), 0);
    std::vector<int> numValid(decomp.getNumRegions(), 0);
    base::SpaceInformationPtr si = getSpaceInformation();
    base::StateValidityCheckerPtr checker = si->getStateValidityChecker();
    base::StateSamplerPtr sampler = si->allocStateSampler();
    base::State *s = si->allocState();
    for (int i = 0; i < NUM_FREEVOL_SAMPLES; ++i)
    {
        sampler->sampleUniform(s);
        int rid = decomp.locateRegion(s);
        if (checker->isValid(s))
            ++numValid[rid];
        ++numTotal[rid];
    }
    si->freeState(s);

    for (int i = 0; i < decomp.getNumRegions(); ++i)
    {
        Region& r = graph[boost::vertex(i, graph)];
        r.volume = decomp.getRegionVolume(i);
        r.percentValidCells = ((double) numValid[i]) / numTotal[i];
        r.freeVolume = r.percentValidCells * r.volume;
    }
}

bool ompl::control::Syclop::updateCoverageEstimate(Region& r, const base::State *s)
{
    const int covCell = covGrid.locateRegion(s);
    if (r.covGridCells.count(covCell) == 1)
        return false;
    r.covGridCells.insert(covCell);
    return true;
}

bool ompl::control::Syclop::updateConnectionEstimate(const Region& c, const Region& d, const base::State *s)
{
    const std::pair<int,int> regions(c.index, d.index);
    Adjacency& adj = regionsToEdge.find(regions)->second;
    const int covCell = covGrid.locateRegion(s);
    if (adj.covGridCells.count(covCell) == 1)
        return false;
    adj.covGridCells.insert(covCell);
    return true;
}

void ompl::control::Syclop::updateRegionEstimates(void)
{
    for (int i = 0; i < decomp.getNumRegions(); ++i)
    {
        Region& r = graph[boost::vertex(i, graph)];
        const double f = r.freeVolume*r.freeVolume*r.freeVolume*r.freeVolume;
        r.alpha = 1 / ((1 + r.covGridCells.size()) * f);
        r.weight = f / ((1 + r.covGridCells.size())*(1 + r.numSelections*r.numSelections));
    }
}

void ompl::control::Syclop::buildGraph(void)
{
    /* The below code builds a boost::graph corresponding to the decomposition.
        It creates Region and Adjacency property objects for each vertex and edge.
        TODO: Correctly initialize the fields for each Region and Adjacency object. */
    VertexIndexMap index = get(boost::vertex_index, graph);
    VertexIter vi, vend;
    std::vector<int> neighbors;
    //Initialize indices before all else
    for (boost::tie(vi,vend) = boost::vertices(graph); vi != vend; ++vi)
    {
        initRegion(graph[*vi]);
        graph[*vi].index = index[*vi];
    }
    for (boost::tie(vi,vend) = boost::vertices(graph); vi != vend; ++vi)
    {
        /* Create an edge between this vertex and each of its neighboring regions in the decomposition,
            and initialize the edge's Adjacency object. */
        decomp.getNeighbors(index[*vi], neighbors);
        for (std::vector<int>::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j)
        {
            RegionGraph::edge_descriptor edge;
            bool ignore;
            boost::tie(edge, ignore) = boost::add_edge(*vi, boost::vertex(*j,graph), graph);
            initEdge(graph[edge], &graph[*vi], &graph[boost::vertex(*j,graph)]);
        }
        neighbors.clear();
    }
}

void ompl::control::Syclop::computeLead(void)
{
    lead.clear();
    if (rng.uniform01() < PROB_SHORTEST_PATH)
    {
        std::vector<RegionGraph::vertex_descriptor> parents(decomp.getNumRegions());
        std::vector<double> distances(decomp.getNumRegions());
        boost::dijkstra_shortest_paths(graph, boost::vertex(startRegion, graph),
            boost::weight_map(get(&Adjacency::cost, graph)).distance_map(
                boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, graph)
            )).predecessor_map(
                boost::make_iterator_property_map(parents.begin(), get(boost::vertex_index, graph))
            )
        );
        int region = goalRegion;
        int leadLength = 1;
        while (region != startRegion)
        {
            region = parents[region];
            ++leadLength;
        }
        lead.resize(leadLength);
        region = goalRegion;
        for (int i = leadLength-1; i >= 0; --i)
        {
            lead[i] = region;
            region = parents[region];
        }
    }
    //TODO Implement random DFS, in case of 1-PROB_SHORTEST_PATH
    for (std::size_t i = 0; i < lead.size()-1; ++i)
    {
        Adjacency& adj = regionsToEdge.find(std::pair<int,int>(lead[i],lead[i+1]))->second;
        if (adj.empty)
        {
            ++adj.numLeadInclusions;
        }
    }
    std::cout << "Computed lead: ";
    for (int i = 0; i < lead.size(); ++i)
        std::cout << lead[i] << " ";
    std::cout << std::endl;
}

int ompl::control::Syclop::selectRegion(void)
{
    const int index = availDist.sample(rng.uniform01());
    Region& region = graph[boost::vertex(index,graph)];
    ++region.numSelections;
    std::cout << "selecting region " << index << std::endl;
    return index;
}

void ompl::control::Syclop::computeAvailableRegions(void)
{
    avail.clear();
    availDist.clear();
    std::cout << "computed avail: ";
    for (int i = lead.size()-1; i >= 0; --i)
    {
        Region& r = graph[boost::vertex(lead[i],graph)];
        if (!r.motions.empty())
        {
            std::cout << lead[i] << " ";
            avail.insert(lead[i]);
            availDist.add(lead[i], r.weight);
            if (rng.uniform01() >= PROB_KEEP_ADDING_TO_AVAIL)
            {
                break;
                //return;
            }
        }
    }
    std::cout << std::endl;
}
