#include "ompl/control/planners/syclop/Syclop.h"

ompl::control::Syclop::Syclop(const SpaceInformationPtr &si, Decomposition &d) :
    ompl::base::Planner(si, "Syclop"), decomp(d), graph(decomp.getNumRegions()), covGrid(COVGRID_LENGTH, 2, d)
{
}

ompl::control::Syclop::~Syclop()
{
}

void ompl::control::Syclop::setup(void)
{
    base::Planner::setup();
    buildGraph();
    //TODO: locate start and goal states
    startRegion = 1;
    goalRegion = 2;
    setupRegionEstimates();
    updateRegionEstimates();
    updateEdgeEstimates();
    printRegions();
    printEdges();

    std::vector<Region*> lead;
    computeLead(lead);
    std::cerr << "Lead: ";
    for (int i = 0; i < lead.size(); ++i)
        std::cerr << lead[i] << " ";
    std::cerr << std::endl;
}

bool ompl::control::Syclop::solve(const base::PlannerTerminationCondition &ptc) {
    return false;
}

void ompl::control::Syclop::printRegions(void) {
    for (int i = 0; i < decomp.getNumRegions(); ++i) {
        Region& r = graph[boost::vertex(i, graph)];
        std::cout << "Region " << r.index << ": ";
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

void ompl::control::Syclop::printEdges(void) {
    EdgeIter ei, end;
    VertexIndexMap index = get(boost::vertex_index, graph);
    for (boost::tie(ei,end) = boost::edges(graph); ei != end; ++ei) {
        const Adjacency& a = graph[*ei];
        std::cout << "Edge (" << index[boost::source(*ei, graph)] << "," << index[boost::target(*ei, graph)] << "): ";
        std::cout << "numCells=" << a.covGridCells.size() << ",";
        std::cout << "nselects=" << a.numSelections << ",";
        std::cout << "cost=" << a.cost << std::endl;
    }
}

void ompl::control::Syclop::initEdge(Adjacency& a, Region* r, Region* s) {
    a.regions.first = r;
    a.regions.second = s;
    a.numSelections = 0;
}

void ompl::control::Syclop::updateEdgeEstimates(void) {
    EdgeIter ei, end;
    VertexIndexMap index = get(boost::vertex_index, graph);
    for (boost::tie(ei,end) = boost::edges(graph); ei != end; ++ei) {
        Adjacency& a = graph[*ei];
        a.cost = (1 + a.numSelections*a.numSelections) / (1 + a.covGridCells.size());
        a.cost *= a.regions.first->alpha * a.regions.second->alpha;
    }
}

void ompl::control::Syclop::initRegion(Region& r) {
    r.numSelections = 0;
    r.volume = 1.0;
    r.percentValidCells = 1.0;
    r.freeVolume = 1.0;
}

void ompl::control::Syclop::setupRegionEstimates(void) {
    std::vector<int> numTotal(decomp.getNumRegions(), 0);
    std::vector<int> numValid(decomp.getNumRegions(), 0);
    base::SpaceInformationPtr si = getSpaceInformation();
    base::StateValidityCheckerPtr checker = si->getStateValidityChecker();
    base::StateSamplerPtr sampler = si->allocStateSampler();
    base::State *s = si->allocState();
    for (int i = 0; i < NUM_FREEVOL_SAMPLES; ++i) {
        sampler->sampleUniform(s);
        int rid = decomp.locateRegion(s);
        if (checker->isValid(s))
            ++numValid[rid];
        ++numTotal[rid];
    }
    si->freeState(s);

    for (int i = 0; i < decomp.getNumRegions(); ++i) {
        Region& r = graph[boost::vertex(i, graph)];
        r.volume = decomp.getRegionVolume(i);
        r.percentValidCells = ((double) numValid[i]) / numTotal[i];
        r.freeVolume = r.percentValidCells * r.volume;
    }
}

void ompl::control::Syclop::updateCoverageEstimate(Region& r, const base::State *s) {
    const int covCell = covGrid.locateRegion(s);
    r.covGridCells.insert(covCell);
}

void ompl::control::Syclop::updateConnectionEstimate(Adjacency& a, const base::State *s) {
    const int covCell = covGrid.locateRegion(s);
    a.covGridCells.insert(covCell);
}

void ompl::control::Syclop::updateRegionEstimates(void) {
    for (int i = 0; i < decomp.getNumRegions(); ++i) {
        Region& r = graph[boost::vertex(i, graph)];
        const double f = r.freeVolume*r.freeVolume*r.freeVolume*r.freeVolume;
        r.alpha = 1 / ((1 + r.covGridCells.size()) * f);
        r.weight = f / ((1 + r.covGridCells.size())*(1 + r.numSelections*r.numSelections));
    }
}

void ompl::control::Syclop::buildGraph(void) {
    /* The below code builds a boost::graph corresponding to the decomposition.
        It creates Region and Adjacency property objects for each vertex and edge.
        TODO: Correctly initialize the fields for each Region and Adjacency object. */
    VertexIndexMap index = get(boost::vertex_index, graph);
    VertexIter vi, vend;
    std::vector<int> neighbors;
    //Iterate over all vertices.
    for (boost::tie(vi,vend) = boost::vertices(graph); vi != vend; ++vi) {
        /* Initialize this vertex's Region object. */
        initRegion(graph[*vi]);
        graph[*vi].index = index[*vi];
        /* Create an edge between this vertex and each of its neighboring regions in the decomposition,
            and initialize the edge's Adjacency object. */
        decomp.getNeighbors(index[*vi], neighbors);
        for (std::vector<int>::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j) {
            RegionGraph::edge_descriptor edge;
            bool ignore;
            boost::tie(edge, ignore) = boost::add_edge(*vi, boost::vertex(*j,graph), graph);
            initEdge(graph[edge], &graph[*vi], &graph[boost::vertex(*j,graph)]);
        }
        neighbors.clear();
    }
}

void ompl::control::Syclop::computeLead(std::vector<Region*>& lead) {
    if (rng.uniform01() < PROB_SHORTEST_PATH) {
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
        while (region != startRegion) {
            region = parents[region];
            ++leadLength;
        }
        lead.resize(leadLength);
        region = goalRegion;
        for (int i = leadLength-1; i >= 0; --i) {
            lead[i] = &graph[boost::vertex(region, graph)];
            region = parents[region];
        }
    }
    //TODO Implement random DFS, in case of 1-PROB_SHORTEST_PATH
}

int ompl::control::Syclop::selectRegion(const std::set<int>& regions) {
    return -1;
}

void ompl::control::Syclop::computeAvailableRegions(const std::vector<Region*>& lead, std::set<Region*>& avail) {
    avail.clear();
    for (int i = lead.size()-1; i >= 0; --i) {
        if (!lead[i]->states.empty()) {
            avail.insert(lead[i]);
            if (rng.uniform01() >= PROB_KEEP_ADDING_TO_AVAIL)
                break;
        }
    }
}
