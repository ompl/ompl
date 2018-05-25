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
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/util/DisableCompilerWarning.h"
#include <limits>
#include <stack>
#include <algorithm>

const double ompl::control::Syclop::Defaults::PROB_ABANDON_LEAD_EARLY = 0.25;
const double ompl::control::Syclop::Defaults::PROB_KEEP_ADDING_TO_AVAIL = 0.50;
const double ompl::control::Syclop::Defaults::PROB_SHORTEST_PATH = 0.95;

void ompl::control::Syclop::setup()
{
    base::Planner::setup();
    if (!leadComputeFn)
        setLeadComputeFn([this](int startRegion, int goalRegion, std::vector<int> &lead)
                         {
                             defaultComputeLead(startRegion, goalRegion, lead);
                         });
    buildGraph();
    addEdgeCostFactor([this](int r, int s)
                      {
                          return defaultEdgeCost(r, s);
                      });
}

void ompl::control::Syclop::clear()
{
    base::Planner::clear();
    lead_.clear();
    availDist_.clear();
    clearEdgeCostFactors();
    clearGraphDetails();
    startRegions_.clear();
    goalRegions_.clear();
}

ompl::base::PlannerStatus ompl::control::Syclop::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    if (!graphReady_)
    {
        numMotions_ = 0;
        setupRegionEstimates();
        setupEdgeEstimates();
        graphReady_ = true;
    }
    while (const base::State *s = pis_.nextStart())
    {
        const int region = decomp_->locateRegion(s);
        startRegions_.insert(region);
        Motion *startMotion = addRoot(s);
        graph_[boost::vertex(region, graph_)].motions.push_back(startMotion);
        ++numMotions_;
        updateCoverageEstimate(graph_[boost::vertex(region, graph_)], s);
    }
    if (startRegions_.empty())
    {
        OMPL_ERROR("%s: There are no valid start states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // We need at least one valid goal sample so that we can find the goal region
    if (goalRegions_.empty())
    {
        if (const base::State *g = pis_.nextGoal(ptc))
            goalRegions_.insert(decomp_->locateRegion(g));
        else
        {
            OMPL_ERROR("%s: Unable to sample a valid goal state", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), numMotions_);

    std::vector<Motion *> newMotions;
    const Motion *solution = nullptr;
    base::Goal *goal = pdef_->getGoal().get();
    double goalDist = std::numeric_limits<double>::infinity();
    bool solved = false;
    while (!ptc && !solved)
    {
        const int chosenStartRegion = startRegions_.sampleUniform();
        int chosenGoalRegion = -1;

        // if we have not sampled too many goal regions already
        if (pis_.haveMoreGoalStates() && goalRegions_.size() < numMotions_ / 2)
        {
            if (const base::State *g = pis_.nextGoal())
            {
                chosenGoalRegion = decomp_->locateRegion(g);
                goalRegions_.insert(chosenGoalRegion);
            }
        }
        if (chosenGoalRegion == -1)
            chosenGoalRegion = goalRegions_.sampleUniform();

        leadComputeFn(chosenStartRegion, chosenGoalRegion, lead_);
        computeAvailableRegions();
        for (int i = 0; i < numRegionExpansions_ && !solved && !ptc; ++i)
        {
            const int region = selectRegion();
            bool improved = false;
            for (int j = 0; j < numTreeSelections_ && !solved && !ptc; ++j)
            {
                newMotions.clear();
                selectAndExtend(graph_[boost::vertex(region, graph_)], newMotions);
                for (std::vector<Motion *>::const_iterator m = newMotions.begin(); m != newMotions.end() && !ptc; ++m)
                {
                    Motion *motion = *m;
                    double distance;
                    solved = goal->isSatisfied(motion->state, &distance);
                    if (solved)
                    {
                        goalDist = distance;
                        solution = motion;
                        break;
                    }

                    // Check for approximate (best-so-far) solution
                    if (distance < goalDist)
                    {
                        goalDist = distance;
                        solution = motion;
                    }
                    const int newRegion = decomp_->locateRegion(motion->state);
                    graph_[boost::vertex(newRegion, graph_)].motions.push_back(motion);
                    ++numMotions_;
                    Region &newRegionObj = graph_[boost::vertex(newRegion, graph_)];
                    improved |= updateCoverageEstimate(newRegionObj, motion->state);
                    /* If tree has just crossed from one region to its neighbor,
                       update the connection estimates. If the tree has crossed an entire region,
                       then region and newRegion are not adjacent, and so we do not update estimates. */
                    if (newRegion != region && regionsToEdge_.count(std::pair<int, int>(region, newRegion)) > 0)
                    {
                        Adjacency *adj = regionsToEdge_[std::pair<int, int>(region, newRegion)];
                        adj->empty = false;
                        ++adj->numSelections;
                        improved |= updateConnectionEstimate(graph_[boost::vertex(region, graph_)], newRegionObj,
                                                             motion->state);
                    }

                    /* If this region already exists in availDist, update its weight. */
                    if (newRegionObj.pdfElem != nullptr)
                        availDist_.update(newRegionObj.pdfElem, newRegionObj.weight);
                    /* Otherwise, only add this region to availDist
                       if it already exists in the lead. */
                    else if (std::find(lead_.begin(), lead_.end(), newRegion) != lead_.end())
                    {
                        PDF<int>::Element *elem = availDist_.add(newRegion, newRegionObj.weight);
                        newRegionObj.pdfElem = elem;
                    }
                }
            }
            if (!improved && rng_.uniform01() < probAbandonLeadEarly_)
                break;
        }
    }
    bool addedSolution = false;
    if (solution != nullptr)
    {
        std::vector<const Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent != nullptr)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, !solved, goalDist, getName());
        addedSolution = true;
    }
    return addedSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::control::Syclop::setLeadComputeFn(const LeadComputeFn &compute)
{
    leadComputeFn = compute;
}

void ompl::control::Syclop::addEdgeCostFactor(const EdgeCostFactorFn &factor)
{
    edgeCostFactors_.push_back(factor);
}

void ompl::control::Syclop::clearEdgeCostFactors()
{
    edgeCostFactors_.clear();
}

void ompl::control::Syclop::initRegion(Region &r)
{
    r.numSelections = 0;
    r.volume = 1.0;
    r.percentValidCells = 1.0;
    r.freeVolume = 1.0;
    r.pdfElem = nullptr;
}

void ompl::control::Syclop::setupRegionEstimates()
{
    std::vector<int> numTotal(decomp_->getNumRegions(), 0);
    std::vector<int> numValid(decomp_->getNumRegions(), 0);
    base::StateValidityCheckerPtr checker = si_->getStateValidityChecker();
    base::StateSamplerPtr sampler = si_->allocStateSampler();
    base::State *s = si_->allocState();

    for (int i = 0; i < numFreeVolSamples_; ++i)
    {
        sampler->sampleUniform(s);
        int rid = decomp_->locateRegion(s);
        if (rid >= 0)
        {
            if (checker->isValid(s))
                ++numValid[rid];
            ++numTotal[rid];
        }
    }
    si_->freeState(s);

    for (int i = 0; i < decomp_->getNumRegions(); ++i)
    {
        Region &r = graph_[boost::vertex(i, graph_)];
        r.volume = decomp_->getRegionVolume(i);
        if (numTotal[i] == 0)
            r.percentValidCells = 1.0;
        else
            r.percentValidCells = ((double)numValid[i]) / (double)numTotal[i];
        r.freeVolume = r.percentValidCells * r.volume;
        if (r.freeVolume < std::numeric_limits<double>::epsilon())
            r.freeVolume = std::numeric_limits<double>::epsilon();
        updateRegion(r);
    }
}

void ompl::control::Syclop::updateRegion(Region &r)
{
    const double f = r.freeVolume * r.freeVolume * r.freeVolume * r.freeVolume;
    r.alpha = 1.0 / ((1 + r.covGridCells.size()) * f);
    r.weight = f / ((1 + r.covGridCells.size()) * (1 + r.numSelections * r.numSelections));
}

void ompl::control::Syclop::initEdge(Adjacency &adj, const Region *source, const Region *target)
{
    adj.source = source;
    adj.target = target;
    updateEdge(adj);
    regionsToEdge_[std::pair<int, int>(source->index, target->index)] = &adj;
}

void ompl::control::Syclop::setupEdgeEstimates()
{
OMPL_PUSH_DISABLE_GCC_WARNING(-Wmaybe-uninitialized)
    EdgeIter ei, eend;
OMPL_POP_GCC
    for (boost::tie(ei, eend) = boost::edges(graph_); ei != eend; ++ei)
    {
        Adjacency &adj = graph_[*ei];
        adj.empty = true;
        adj.numLeadInclusions = 0;
        adj.numSelections = 0;
        updateEdge(adj);
    }
}

void ompl::control::Syclop::updateEdge(Adjacency &a)
{
    a.cost = 1.0;
    for (const auto &factor : edgeCostFactors_)
    {
        a.cost *= factor(a.source->index, a.target->index);
    }
}

bool ompl::control::Syclop::updateCoverageEstimate(Region &r, const base::State *s)
{
    const int covCell = covGrid_.locateRegion(s);
    if (r.covGridCells.count(covCell) == 1)
        return false;
    r.covGridCells.insert(covCell);
    updateRegion(r);
    return true;
}

bool ompl::control::Syclop::updateConnectionEstimate(const Region &c, const Region &d, const base::State *s)
{
    Adjacency &adj = *regionsToEdge_[std::pair<int, int>(c.index, d.index)];
    const int covCell = covGrid_.locateRegion(s);
    if (adj.covGridCells.count(covCell) == 1)
        return false;
    adj.covGridCells.insert(covCell);
    updateEdge(adj);
    return true;
}

void ompl::control::Syclop::buildGraph()
{
    VertexIndexMap index = get(boost::vertex_index, graph_);
    std::vector<int> neighbors;
    for (int i = 0; i < decomp_->getNumRegions(); ++i)
    {
        const RegionGraph::vertex_descriptor v = boost::add_vertex(graph_);
        Region &r = graph_[boost::vertex(v, graph_)];
        initRegion(r);
        r.index = index[v];
    }
    VertexIter vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(graph_); vi != vend; ++vi)
    {
        /* Create an edge between this vertex and each of its neighboring regions in the decomposition,
            and initialize the edge's Adjacency object. */
        decomp_->getNeighbors(index[*vi], neighbors);
        for (const auto &j : neighbors)
        {
            RegionGraph::edge_descriptor edge;
            bool ignore;
            boost::tie(edge, ignore) = boost::add_edge(*vi, boost::vertex(j, graph_), graph_);
            initEdge(graph_[edge], &graph_[*vi], &graph_[boost::vertex(j, graph_)]);
        }
        neighbors.clear();
    }
}

void ompl::control::Syclop::clearGraphDetails()
{
    VertexIter vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(graph_); vi != vend; ++vi)
        graph_[*vi].clear();
OMPL_PUSH_DISABLE_GCC_WARNING(-Wmaybe-uninitialized)
    EdgeIter ei, eend;
OMPL_POP_GCC
    for (boost::tie(ei, eend) = boost::edges(graph_); ei != eend; ++ei)
        graph_[*ei].clear();
    graphReady_ = false;
}

int ompl::control::Syclop::selectRegion()
{
    const int index = availDist_.sample(rng_.uniform01());
    Region &region = graph_[boost::vertex(index, graph_)];
    ++region.numSelections;
    updateRegion(region);
    return index;
}

void ompl::control::Syclop::computeAvailableRegions()
{
    for (unsigned int i = 0; i < availDist_.size(); ++i)
        graph_[boost::vertex(availDist_[i], graph_)].pdfElem = nullptr;
    availDist_.clear();
    for (int i = lead_.size() - 1; i >= 0; --i)
    {
        Region &r = graph_[boost::vertex(lead_[i], graph_)];
        if (!r.motions.empty())
        {
            r.pdfElem = availDist_.add(lead_[i], r.weight);
            if (rng_.uniform01() >= probKeepAddingToAvail_)
                break;
        }
    }
}

void ompl::control::Syclop::defaultComputeLead(int startRegion, int goalRegion, std::vector<int> &lead)
{
    lead.clear();
    if (startRegion == goalRegion)
    {
        lead.push_back(startRegion);
        return;
    }

    if (rng_.uniform01() < probShortestPath_)
    {
        std::vector<RegionGraph::vertex_descriptor> parents(decomp_->getNumRegions());
        std::vector<double> distances(decomp_->getNumRegions());

        try
        {
            boost::astar_search(graph_, boost::vertex(startRegion, graph_),
                                DecompositionHeuristic(this, getRegionFromIndex(goalRegion)),
                                boost::weight_map(get(&Adjacency::cost, graph_))
                                    .distance_map(boost::make_iterator_property_map(distances.begin(),
                                                                                    get(boost::vertex_index, graph_)))
                                    .predecessor_map(boost::make_iterator_property_map(
                                        parents.begin(), get(boost::vertex_index, graph_)))
                                    .visitor(GoalVisitor(goalRegion)));
        }
        catch (found_goal fg)
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
            for (int i = leadLength - 1; i >= 0; --i)
            {
                lead[i] = region;
                region = parents[region];
            }
        }
    }
    else
    {
        /* Run a random-DFS over the decomposition graph from the start region to the goal region.
           There may be a way to do this using boost::depth_first_search. */
        VertexIndexMap index = get(boost::vertex_index, graph_);
        std::stack<int> nodesToProcess;
        std::vector<int> parents(decomp_->getNumRegions(), -1);
        parents[startRegion] = startRegion;
        nodesToProcess.push(startRegion);
        bool goalFound = false;
        while (!goalFound && !nodesToProcess.empty())
        {
            const int v = nodesToProcess.top();
            nodesToProcess.pop();
            std::vector<int> neighbors;
            boost::graph_traits<RegionGraph>::adjacency_iterator ai, aend;
            for (boost::tie(ai, aend) = adjacent_vertices(boost::vertex(v, graph_), graph_); ai != aend; ++ai)
            {
                if (parents[index[*ai]] < 0)
                {
                    neighbors.push_back(index[*ai]);
                    parents[index[*ai]] = v;
                }
            }
            for (std::size_t i = 0; i < neighbors.size(); ++i)
            {
                const int choice = rng_.uniformInt(i, neighbors.size() - 1);
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
                    for (int j = leadLength - 1; j >= 0; --j)
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

    // Now that we have a lead, update the edge weights.
    for (std::size_t i = 0; i < lead.size() - 1; ++i)
    {
        Adjacency &adj = *regionsToEdge_[std::pair<int, int>(lead[i], lead[i + 1])];
        if (adj.empty)
        {
            ++adj.numLeadInclusions;
            updateEdge(adj);
        }
    }
}

double ompl::control::Syclop::defaultEdgeCost(int r, int s)
{
    const Adjacency &a = *regionsToEdge_[std::pair<int, int>(r, s)];
    double factor = 1.0;
    const int nsel = (a.empty ? a.numLeadInclusions : a.numSelections);
    factor = (1.0 + (double)nsel * nsel) / (1.0 + (double)a.covGridCells.size() * a.covGridCells.size());
    factor *= (a.source->alpha * a.target->alpha);
    return factor;
}
