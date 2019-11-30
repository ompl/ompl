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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Ioan Sucan, James D. Marble, Ryan Luna, Henning Kayser */

#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <thread>
#include <typeinfo>

#include "GoalVisitor.hpp"

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }  // namespace magic
}  // namespace ompl

ompl::geometric::PRM::PRM(const base::SpaceInformationPtr &si, bool starStrategy)
  : base::Planner(si, "PRM")
  , starStrategy_(starStrategy)
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &PRM::setMaxNearestNeighbors,
                                            &PRM::getMaxNearestNeighbors, std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationCount(); });
    addPlannerProgressProperty("best cost REAL", [this] { return getBestCost(); });
    addPlannerProgressProperty("milestone count INTEGER", [this] { return getMilestoneCountString(); });
    addPlannerProgressProperty("edge count INTEGER", [this] { return getEdgeCountString(); });
}

ompl::geometric::PRM::PRM(const base::PlannerData &data, bool starStrategy)
  : PRM(data.getSpaceInformation(), starStrategy)
{
    if (data.numVertices() > 0)
    {
        // mapping between vertex id from PlannerData and Vertex in Boost.Graph
        std::map<unsigned int, Vertex> vertices;
        // helper function to create vertices as needed and update the vertices mapping
        const auto &getOrCreateVertex = [&](unsigned int vertex_index) {
            if (!vertices.count(vertex_index))
            {
                const auto &data_vertex = data.getVertex(vertex_index);
                Vertex graph_vertex = boost::add_vertex(g_);
                stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
                totalConnectionAttemptsProperty_[graph_vertex] = 1;
                successfulConnectionAttemptsProperty_[graph_vertex] = 0;
                vertices[vertex_index] = graph_vertex;
            }
            return vertices.at(vertex_index);
        };

        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

        for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
        {
            Vertex m = getOrCreateVertex(vertex_index);
            std::vector<unsigned int> neighbor_indices;
            data.getEdges(vertex_index, neighbor_indices);
            if (neighbor_indices.empty())
            {
                disjointSets_.make_set(m);
            }
            else
            {
                for (const unsigned int neighbor_index : neighbor_indices)
                {
                    Vertex n = getOrCreateVertex(neighbor_index);
                    totalConnectionAttemptsProperty_[n]++;
                    successfulConnectionAttemptsProperty_[n]++;
                    base::Cost weight;
                    data.getEdgeWeight(vertex_index, neighbor_index, &weight);
                    const Graph::edge_property_type properties(weight);
                    boost::add_edge(m, n, properties, g_);
                    uniteComponents(m, n);
                }
            }
            nn_->add(m);
        }
    }
}

ompl::geometric::PRM::~PRM()
{
    freeMemory();
}

void ompl::geometric::PRM::setup()
{
    Planner::setup();
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!connectionStrategy_)
        setDefaultConnectionStrategy();
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &) { return true; };

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void ompl::geometric::PRM::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = KStrategy<Vertex>(k, nn_);
    if (isSetup())
        setup();
}

unsigned int ompl::geometric::PRM::getMaxNearestNeighbors() const
{
    const auto strategy = connectionStrategy_.target<KStrategy<Vertex>>();
    return strategy ? strategy->getNumNeighbors() : 0u;
}

void ompl::geometric::PRM::setDefaultConnectionStrategy()
{
    if (starStrategy_)
        connectionStrategy_ = KStarStrategy<Vertex>([this] { return milestoneCount(); }, nn_, si_->getStateDimension());
    else
        connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
}

void ompl::geometric::PRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::PRM::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::PRM::clear()
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::PRM::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

void ompl::geometric::PRM::expandRoadmap(double expandTime)
{
    expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
}

void ompl::geometric::PRM::expandRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> states(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(ptc, states);
    si_->freeStates(states);
}

void ompl::geometric::PRM::expandRoadmap(const base::PlannerTerminationCondition &ptc,
                                         std::vector<base::State *> &workStates)
{
    // construct a probability distribution over the vertices in the roadmap
    // as indicated in
    //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
    //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

    PDF<Vertex> pdf;
    foreach (Vertex v, boost::vertices(g_))
    {
        const unsigned long int t = totalConnectionAttemptsProperty_[v];
        pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
    }

    if (pdf.empty())
        return;

    while (!ptc)
    {
        iterations_++;
        Vertex v = pdf.sample(rng_.uniform01());
        unsigned int s =
            si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
        if (s > 0)
        {
            s--;
            Vertex last = addMilestone(si_->cloneState(workStates[s]));

            graphMutex_.lock();
            for (unsigned int i = 0; i < s; ++i)
            {
                // add the vertex along the bouncing motion
                Vertex m = boost::add_vertex(g_);
                stateProperty_[m] = si_->cloneState(workStates[i]);
                totalConnectionAttemptsProperty_[m] = 1;
                successfulConnectionAttemptsProperty_[m] = 0;
                disjointSets_.make_set(m);

                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, m, properties, g_);
                uniteComponents(v, m);

                // add the vertex to the nearest neighbors data structure
                nn_->add(m);
                v = m;
            }

            // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
            // we add an edge
            if (s > 0 || !sameComponent(v, last))
            {
                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, last, properties, g_);
                uniteComponents(v, last);
            }
            graphMutex_.unlock();
        }
    }
}

void ompl::geometric::PRM::growRoadmap(double growTime)
{
    growRoadmap(base::timedPlannerTerminationCondition(growTime));
}

void ompl::geometric::PRM::growRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    base::State *workState = si_->allocState();
    growRoadmap(ptc, workState);
    si_->freeState(workState);
}

void ompl::geometric::PRM::growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState)
{
    /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
    while (!ptc)
    {
        iterations_++;
        // search for a valid state
        bool found = false;
        while (!found && !ptc)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(workState);
                attempts++;
            } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
        }
        // add it as a milestone
        if (found)
            addMilestone(si_->cloneState(workState));
    }
}

void ompl::geometric::PRM::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
    auto *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    while (!ptc && !addedNewSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st != nullptr)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        // Check for a solution
        addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
        // Sleep for 1ms
        if (!addedNewSolution_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool ompl::geometric::PRM::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                                  base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    base::Cost sol_cost(opt_->infiniteCost());
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                base::PathPtr p = constructSolution(start, goal);
                if (p)
                {
                    base::Cost pathCost = p->cost(opt_);
                    if (opt_->isCostBetterThan(pathCost, bestCost_))
                        bestCost_ = pathCost;
                    // Check if optimization objective is satisfied
                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        return true;
                    }
                    if (opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                    }
                }
            }
        }
    }

    return false;
}

bool ompl::geometric::PRM::addedNewSolution() const
{
    return addedNewSolution_;
}

ompl::base::PlannerStatus ompl::geometric::PRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    // Reset addedNewSolution_ member and create solution checking thread
    addedNewSolution_ = false;
    base::PathPtr sol;
    std::thread slnThread([this, &ptc, &sol] { checkForSolution(ptc, sol); });

    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || addedNewSolution(); });

    constructRoadmap(ptcOrSolutionFound);

    // Ensure slnThread is ceased before exiting solve
    slnThread.join();

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);
    }
    else
    {
        // Return an approximate solution.
        ompl::base::Cost diff = constructApproximateSolution(startM_, goalM_, sol);
        if (!opt_->isFinite(diff))
        {
            OMPL_INFORM("Closest path is still start and goal");
            return base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("Using approximate solution, heuristic cost-to-go is %f", diff.value());
        pdef_->addSolutionPath(sol, true, diff.value(), getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::PRM::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    bestCost_ = opt_->infiniteCost();
    while (!ptc())
    {
        // maintain a 2:1 ratio for growing/expansion of roadmap
        // call growRoadmap() twice as long for every call of expandRoadmap()
        if (grow)
            growRoadmap(base::plannerOrTerminationCondition(
                            ptc, base::timedPlannerTerminationCondition(2.0 * magic::ROADMAP_BUILD_TIME)),
                        xstates[0]);
        else
            expandRoadmap(base::plannerOrTerminationCondition(
                              ptc, base::timedPlannerTerminationCondition(magic::ROADMAP_BUILD_TIME)),
                          xstates);
        grow = !grow;
    }

    si_->freeStates(xstates);
}

ompl::geometric::PRM::Vertex ompl::geometric::PRM::addMilestone(base::State *state)
{
    std::lock_guard<std::mutex> _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(n, m))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(n, m, properties, g_);
                uniteComponents(n, m);
            }
        }

    nn_->add(m);

    return m;
}

void ompl::geometric::PRM::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool ompl::geometric::PRM::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

ompl::base::Cost ompl::geometric::PRM::constructApproximateSolution(const std::vector<Vertex> &starts,
                                                                    const std::vector<Vertex> &goals,
                                                                    base::PathPtr &solution)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    base::Goal *g = pdef_->getGoal().get();
    base::Cost closestVal(opt_->infiniteCost());
    bool approxPathJustStart = true;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            base::Cost heuristicCost(costHeuristic(start, goal));
            if (opt_->isCostBetterThan(heuristicCost, closestVal))
            {
                closestVal = heuristicCost;
                approxPathJustStart = true;
            }
            if (!g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                continue;
            }
            base::PathPtr p;
            boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> dist(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> rank(boost::num_vertices(g_));

            try
            {
                // Consider using a persistent distance_map if it's slow
                boost::astar_search(
                    g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
                    boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare(
                            [this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(goal)));
            }
            catch (AStarFoundGoal &)
            {
            }

            Vertex closeToGoal = start;
            for (auto vp = vertices(g_); vp.first != vp.second; vp.first++)
            {
                // We want to get the distance of each vertex to the goal.
                // Boost lets us get cost-to-come, cost-to-come+dist-to-goal,
                // but not just dist-to-goal.
                ompl::base::Cost dist_to_goal(costHeuristic(*vp.first, goal));
                if (opt_->isFinite(rank[*vp.first]) && opt_->isCostBetterThan(dist_to_goal, closestVal))
                {
                    closeToGoal = *vp.first;
                    closestVal = dist_to_goal;
                    approxPathJustStart = false;
                }
            }
            if (closeToGoal != start)
            {
                auto p(std::make_shared<PathGeometric>(si_));
                for (Vertex pos = closeToGoal; prev[pos] != pos; pos = prev[pos])
                    p->append(stateProperty_[pos]);
                p->append(stateProperty_[start]);
                p->reverse();

                solution = p;
            }
        }
    }
    if (approxPathJustStart)
    {
        return opt_->infiniteCost();
    }
    return closestVal;
}

ompl::base::PathPtr ompl::geometric::PRM::constructSolution(const Vertex &start, const Vertex &goal)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
            g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev)
                .distance_compare([this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                .distance_inf(opt_->infiniteCost())
                .distance_zero(opt_->identityCost())
                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    auto p(std::make_shared<PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
        p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    return p;
}

void ompl::geometric::PRM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (unsigned long i : startM_)
        data.addStartVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<PRM *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : goalM_)
        data.addGoalVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<PRM *>(this)->disjointSets_.find_set(i)));

    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]), base::PlannerDataVertex(stateProperty_[v1]));

        // Add tags for the newly added vertices
        data.tagState(stateProperty_[v1], const_cast<PRM *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2], const_cast<PRM *>(this)->disjointSets_.find_set(v2));
    }
}

ompl::base::Cost ompl::geometric::PRM::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}
