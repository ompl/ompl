/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Matt Maly */

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/base/GoalState.h"
#include "ompl/base/ProblemDefinition.h"
#include <limits>
#include <stack>
#include <boost/math/special_functions/fpclassify.hpp>

void ompl::control::Syclop::setup(void)
{
    base::Planner::setup();
    buildGraph();
}

void ompl::control::Syclop::clear(void)
{
    base::Planner::clear();
    lead.clear();
    avail.clear();
    availDist.clear();
    clearGraphDetails();
}

bool ompl::control::Syclop::solve(const base::PlannerTerminationCondition& ptc)
{
    checkValidity();
    if (!graphReady)
        initGraph();
    std::set<Motion*> newMotions;
    base::Goal* goal = getProblemDefinition()->getGoal().get();
    Motion* solution = NULL;
    Motion* approxSoln = NULL;
    double goalDist = std::numeric_limits<double>::infinity();
    bool solved = false;
    while (!ptc() && !solved)
    {
        computeLead();
        computeAvailableRegions();
        for (int i = 0; i < NUM_AVAIL_EXPLORATIONS && !solved; ++i)
        {
            const int region = selectRegion();
            bool improved = false;
            for (int j = 0; j < NUM_TREE_SELECTIONS && !solved; ++j)
            {
                newMotions.clear();
                selectAndExtend(graph[boost::vertex(region,graph)], newMotions);
                for (std::set<Motion*>::const_iterator m = newMotions.begin(); m != newMotions.end(); ++m)
                {
                    Motion* motion = *m;
                    base::State* state = motion->state;
                    double distance;
                    solved = goal->isSatisfied(state, &distance);
                    if (solved)
                    {
                        goalDist = distance;
                        solution = motion;
                        break;
                    }
                    else if (distance < goalDist)
                    {
                        goalDist = distance;
                        approxSoln = motion;
                    }
                    const int oldRegion = decomp.locateRegion(motion->parent->state);
                    const int newRegion = decomp.locateRegion(state);
                    graph[boost::vertex(newRegion,graph)].motions.push_back(motion);
                    if (newRegion != oldRegion)
                    {
                        avail.insert(newRegion);
                        /* If the tree crosses an entire region and creates an edge (u,v) for which Proj(u) and Proj(v) are non-neighboring regions,
                            then we do not update connection estimates. This is because Syclop's shortest-path lead computation only considers neighboring regions. */
                        std::map<std::pair<int,int>, Adjacency*>::iterator i = regionsToEdge.find(std::pair<int,int>(oldRegion,newRegion));
                        if (i != regionsToEdge.end())
                        {
                            Adjacency& adj = *regionsToEdge.find(std::pair<int,int>(oldRegion,newRegion))->second;
                            adj.empty = false;
                            ++adj.numSelections;
                            improved |= updateConnectionEstimate(graph[boost::vertex(oldRegion,graph)], graph[boost::vertex(newRegion,graph)], state);
                        }
                    }
                    improved |= updateCoverageEstimate(graph[boost::vertex(newRegion, graph)], state);
                }
            }
            if (!improved && rng.uniform01() < PROB_ABANDON_LEAD_EARLY)
                break;
        }
    }

    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxSoln;
        approximate = true;
    }

    if (solution != NULL)
    {
        std::vector<const Motion*> mpath;
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
        goal->setDifference(goalDist);
        goal->setSolutionPath(base::PathPtr(path), approximate);

        if (approximate)
            msg_.warn("Found approximate solution");
    }
    return goal->isAchieved();
}

void ompl::control::Syclop::addEdgeCostFactor(const EdgeCostFactorFn& factor)
{
    edgeCostFactors.push_back(factor);
}

void ompl::control::Syclop::clearEdgeCostFactors(void)
{
    edgeCostFactors.clear();
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
    base::StateValidityCheckerPtr checker = si_->getStateValidityChecker();
    base::StateSamplerPtr sampler = si_->allocStateSampler();
    base::State* s = si_->allocState();

    for (int i = 0; i < NUM_FREEVOL_SAMPLES; ++i)
    {
        sampler->sampleUniform(s);
        int rid = decomp.locateRegion(s);
        if (checker->isValid(s))
            ++numValid[rid];
        ++numTotal[rid];
    }
    si_->freeState(s);

    for (int i = 0; i < decomp.getNumRegions(); ++i)
    {
        Region& r = graph[boost::vertex(i, graph)];
        r.volume = decomp.getRegionVolume(i);
        r.percentValidCells = ((double) numValid[i]) / numTotal[i];
        r.freeVolume = r.percentValidCells * r.volume;
        updateRegion(r);
    }
}

void ompl::control::Syclop::updateRegion(Region& r)
{
    const double f = r.freeVolume*r.freeVolume*r.freeVolume*r.freeVolume;
    r.alpha = 1 / ((1 + r.covGridCells.size()) * f);
    r.weight = f / ((1 + r.covGridCells.size())*(1 + r.numSelections*r.numSelections));
}

void ompl::control::Syclop::initEdge(Adjacency& adj, Region* r, Region* s)
{
    adj.source = r;
    adj.target = s;
    updateEdge(adj);
    std::pair<int,int> regions(r->index, s->index);
    std::pair<std::pair<int,int>,Adjacency*> mapping(regions, &adj);
    regionsToEdge.insert(mapping);
}

void ompl::control::Syclop::setupEdgeEstimates(void)
{
    EdgeIter ei, eend;
    for (boost::tie(ei,eend) = boost::edges(graph); ei != eend; ++ei)
    {
        Adjacency& adj = graph[*ei];
        adj.empty = true;
        adj.numLeadInclusions = 0;
        adj.numSelections = 0;
        updateEdge(adj);
    }
}

void ompl::control::Syclop::updateEdge(Adjacency& a)
{
    const double nsel = (a.empty ? a.numLeadInclusions : a.numSelections);
    a.cost = (1 + nsel*nsel) / (1 + a.covGridCells.size()*a.covGridCells.size());
    a.cost *= a.source->alpha * a.target->alpha;
    for (std::vector<EdgeCostFactorFn>::const_iterator i = edgeCostFactors.begin(); i != edgeCostFactors.end(); ++i)
    {
        const EdgeCostFactorFn& factor = *i;
        a.cost *= factor(a.source->index, a.target->index);
    }
}

bool ompl::control::Syclop::updateCoverageEstimate(Region& r, const base::State *s)
{
    const int covCell = covGrid.locateRegion(s);
    if (r.covGridCells.count(covCell) == 1)
        return false;
    r.covGridCells.insert(covCell);
    updateRegion(r);
    return true;
}

bool ompl::control::Syclop::updateConnectionEstimate(const Region& c, const Region& d, const base::State *s)
{
    const std::pair<int,int> regions(c.index, d.index);
    Adjacency& adj = *regionsToEdge.find(regions)->second;
    const int covCell = covGrid.locateRegion(s);
    if (adj.covGridCells.count(covCell) == 1)
        return false;
    adj.covGridCells.insert(covCell);
    updateEdge(adj);
    return true;
}

void ompl::control::Syclop::buildGraph(void)
{
    /* The below code builds a boost::graph corresponding to the decomposition.
        It creates Region and Adjacency property objects for each vertex and edge. */
    VertexIndexMap index = get(boost::vertex_index, graph);
    std::vector<int> neighbors;
    for (int i = 0; i < decomp.getNumRegions(); ++i)
    {
        const RegionGraph::vertex_descriptor v = boost::add_vertex(graph);
        Region& r = graph[boost::vertex(v,graph)];
        initRegion(r);
        r.index = index[v];
    }
    VertexIter vi, vend;
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

void ompl::control::Syclop::initGraph(void)
{
    const base::ProblemDefinitionPtr& pdef = getProblemDefinition();
    /* TODO: Handle multiple start states. */
    base::State* start = pdef->getStartState(0);
    startRegion = decomp.locateRegion(start);
    /* Here we are assuming that we have a GoalSampleableRegion. */
    base::State* goal = si_->allocState();
    pdef->getGoal()->as<base::GoalSampleableRegion>()->sampleGoal(goal);
    goalRegion = decomp.locateRegion(goal);
    si_->freeState(goal);
    Motion* startMotion = initializeTree(start);
    graph[boost::vertex(startRegion,graph)].motions.push_back(startMotion);
    setupRegionEstimates();
    setupEdgeEstimates();
    updateCoverageEstimate(graph[boost::vertex(startRegion,graph)], start);
    graphReady = true;
}

void ompl::control::Syclop::clearGraphDetails(void)
{
    VertexIter vi, vend;
    for (boost::tie(vi,vend) = boost::vertices(graph); vi != vend; ++vi)
        graph[*vi].clear();
    EdgeIter ei, eend;
    for (boost::tie(ei,eend) = boost::edges(graph); ei != eend; ++ei)
        graph[*ei].clear();
    graphReady = false;
}

void ompl::control::Syclop::computeLead(void)
{
    /* For now, this function assumes that a path exists in the decomposition
     * from startRegion to goalRegion. */
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
    else
    {
        VertexIndexMap index = get(boost::vertex_index, graph);
        std::stack<int> nodesToProcess;
        std::vector<int> parents(decomp.getNumRegions(), -1);
        parents[startRegion] = startRegion;
        nodesToProcess.push(startRegion);
        bool goalFound = false;
        while (!goalFound && !nodesToProcess.empty())
        {
            const int v = nodesToProcess.top();
            nodesToProcess.pop();
            std::vector<int> neighbors;
            boost::graph_traits<RegionGraph>::adjacency_iterator ai, aend;
            for (boost::tie(ai,aend) = adjacent_vertices(boost::vertex(v,graph),graph); ai != aend; ++ai)
            {
                if (parents[index[*ai]] < 0)
                {
                    neighbors.push_back(index[*ai]);
                    parents[index[*ai]] = v;
                }
            }
            for (std::size_t i = 0; i < neighbors.size(); ++i)
            {
                const int choice = rng.uniformInt(i, neighbors.size()-1);
                if (neighbors[choice] == goalRegion)
                {
                    int region = goalRegion;
                    int leadLength = 1;
                    while (region != startRegion)
                    {
                        region = parents[region];
                        ++leadLength;
                    }
                    lead.resize(leadLength);
                    region = goalRegion;
                    for (int j = leadLength-1; j >= 0; --j)
                    {
                        lead[j] = region;
                        region = parents[region];
                    }
                    goalFound = true;
                    break;
                }
                nodesToProcess.push(neighbors[choice]);
                std::swap(neighbors[i], neighbors[choice]);
            }
        }
    }
    for (std::size_t i = 0; i < lead.size()-1; ++i)
    {
        Adjacency& adj = *regionsToEdge.find(std::pair<int,int>(lead[i],lead[i+1]))->second;
        if (adj.empty)
        {
            ++adj.numLeadInclusions;
            updateEdge(adj);
        }
    }
}

int ompl::control::Syclop::selectRegion(void)
{
    const int index = availDist.sample(rng.uniform01());
    Region& region = graph[boost::vertex(index,graph)];
    ++region.numSelections;
    updateRegion(region);
    return index;
}

void ompl::control::Syclop::computeAvailableRegions(void)
{
    avail.clear();
    availDist.clear();
    for (int i = lead.size()-1; i >= 0; --i)
    {
        Region& r = graph[boost::vertex(lead[i],graph)];
        if (!r.motions.empty())
        {
            avail.insert(lead[i]);
            availDist.add(lead[i], r.weight);
            if (rng.uniform01() >= PROB_KEEP_ADDING_TO_AVAIL)
                break;
        }
    }
}
