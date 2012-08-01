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

/* Author: Ioan Sucan, James D. Marble */

#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/PDF.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

namespace ompl
{
    namespace magic
    {

        /** \brief Maximum number of sampling attempts to find a valid state,
            without checking whether the allowed time elapsed. This value
            should not really be changed. */
        static const unsigned int FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK = 2;

        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;
    }
}

ompl::geometric::PRM::PRM(const base::SpaceInformationPtr &si, bool starStrategy) :
    base::Planner(si, "PRM"),
    starStrategy_(starStrategy),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
    successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    edgeIDProperty_(boost::get(boost::edge_index, g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    maxEdgeID_(0),
    userSetConnectionStrategy_(false),
    addedSolution_(false)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &PRM::setMaxNearestNeighbors);
}

ompl::geometric::PRM::~PRM(void)
{
    freeMemory();
}

void ompl::geometric::PRM::setup(void)
{
    Planner::setup();
    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Vertex>());
    nn_->setDistanceFunction(boost::bind(&PRM::distanceFunction, this, _1, _2));
    if (!connectionStrategy_)
    {
        if (starStrategy_)
            connectionStrategy_ = KStarStrategy<Vertex>(boost::bind(&PRM::milestoneCount, this), nn_, si_->getStateDimension());
        else
            connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
    }
    if (!connectionFilter_)
        connectionFilter_ = boost::lambda::constant(true);
}

void ompl::geometric::PRM::setMaxNearestNeighbors(unsigned int k)
{
    if (!setup_)
        setup();
    connectionStrategy_ = KStrategy<Vertex>(k, nn_);
}

void ompl::geometric::PRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::PRM::clearQuery(void)
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::PRM::clear(void)
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();
    maxEdgeID_ = 0;
}

void ompl::geometric::PRM::freeMemory(void)
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

    std::vector<base::State*> states(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(ptc, states);
    si_->freeStates(states);
}

void ompl::geometric::PRM::expandRoadmap(const base::PlannerTerminationCondition &ptc,
                                         std::vector<base::State*> &workStates)
{
    // construct a probability distribution over the vertices in the roadmap
    // as indicated in
    //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
    //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

    PDF<Vertex> pdf;
    foreach (Vertex v, boost::vertices(g_))
    {
        const unsigned int t = totalConnectionAttemptsProperty_[v];
        pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
    }

    if (pdf.empty())
        return;

    while (ptc() == false)
    {
        Vertex v = pdf.sample(rng_.uniform01());
        unsigned int s = si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
        if (s > 0)
        {
            s--;
            Vertex last = addMilestone(si_->cloneState(workStates[s]));

            graphMutex_.lock();
            for (unsigned int i = 0 ; i < s ; ++i)
            {
                // add the vertex along the bouncing motion
                Vertex m = boost::add_vertex(g_);
                stateProperty_[m] = si_->cloneState(workStates[i]);
                totalConnectionAttemptsProperty_[m] = 1;
                successfulConnectionAttemptsProperty_[m] = 0;
                disjointSets_.make_set(m);

                // add the edge to the parent vertex
                const double weight = distanceFunction(v, m);
                const unsigned int id = maxEdgeID_++;
                const Graph::edge_property_type properties(weight, id);
                boost::add_edge(v, m, properties, g_);
                uniteComponents(v, m);

                // add the vertex to the nearest neighbors data structure
                nn_->add(m);
                v = m;
            }

            // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
            // we add an edge
            if (s > 0 || !boost::same_component(v, last, disjointSets_))
            {
                // add the edge to the parent vertex
                const double weight = distanceFunction(v, last);
                const unsigned int id = maxEdgeID_++;
                const Graph::edge_property_type properties(weight, id);
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
    growRoadmap (ptc, workState);
    si_->freeState(workState);
}

void ompl::geometric::PRM::growRoadmap(const base::PlannerTerminationCondition &ptc,
                                       base::State *workState)
{
    while (ptc() == false)
    {
        // search for a valid state
        bool found = false;
        while (!found && ptc() == false)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(workState);
                attempts++;
            } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK && !found);
        }
        // add it as a milestone
        if (found)
            addMilestone(si_->cloneState(workState));
    }
}

void ompl::geometric::PRM::checkForSolution (const base::PlannerTerminationCondition &ptc,
                                             base::PathPtr &solution)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    while (!ptc() && !addedSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        // Check for a solution
        addedSolution_ = haveSolution (startM_, goalM_, solution);
        // Sleep for 1ms
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}

bool ompl::geometric::PRM::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    double sol_cost;
    bool sol_cost_set = false;
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            if (boost::same_component(start, goal, disjointSets_) &&
                g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                // If there is an optimization objective, check it
                if (pdef_->hasOptimizationObjective())
                {
                    base::PathPtr p = constructSolution(start, goal);
                    double obj_cost = pdef_->getOptimizationObjective()->getCost(p);
                    if (pdef_->getOptimizationObjective()->isSatisfied(obj_cost)) // Sufficient solution
                    {
                        solution = p;
                        return true;
                    }
                    else
                    {          
                        if (solution && !sol_cost_set)
                        {
                            sol_cost = pdef_->getOptimizationObjective()->getCost(solution);
                            sol_cost_set = true;
                        }
                      
                        if (!solution || obj_cost < sol_cost)
                        {
                            solution = p;
                            sol_cost = obj_cost;   
                            sol_cost_set = true;
                        }
                    }
                }
                else // Accept the solution, regardless of cost
                {
                    solution = constructSolution(start, goal);
                    return true;
                }
            }
        }
    }

    return false;
}

bool ompl::geometric::PRM::addedNewSolution(void) const
{
    return addedSolution_;
}

ompl::base::PlannerStatus ompl::geometric::PRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        logError("Goal undefined or unknown type of goal");
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.size() == 0)
    {
        logError("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        logError("Insufficient states in sampleable goal region");
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            logError("Unable to find any valid goal states");
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    unsigned int nrStartStates = boost::num_vertices(g_);
    logInform("Starting with %u states", nrStartStates);

    std::vector<base::State*> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    // Reset addedSolution_ member and create solution checking thread
    addedSolution_ = false;
    base::PathPtr sol;
    sol.reset();
    boost::thread slnThread (boost::bind(&PRM::checkForSolution, this, ptc, boost::ref(sol)));

    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerOrTerminationCondition ptcOrSolutionFound (ptc, base::PlannerTerminationCondition(boost::bind(&PRM::addedNewSolution, this)));

    while (ptcOrSolutionFound() == false)
    {
        // maintain a 2:1 ratio for growing/expansion of roadmap
        // call growRoadmap() twice as long for every call of expandRoadmap()
        if (grow)
            growRoadmap(base::PlannerOrTerminationCondition(ptcOrSolutionFound, base::timedPlannerTerminationCondition(2.0*magic::ROADMAP_BUILD_TIME)), xstates[0]);
        else
            expandRoadmap(base::PlannerOrTerminationCondition(ptcOrSolutionFound, base::timedPlannerTerminationCondition(magic::ROADMAP_BUILD_TIME)), xstates);
        grow = !grow;
    }

    // Ensure slnThread is ceased before exiting solve
    slnThread.join();

    logInform("Created %u states", boost::num_vertices(g_) - nrStartStates);

    if (sol)
    {
        if (addedNewSolution())
            pdef_->addSolutionPath (sol);
        else
            // the solution is exact, but not as short as we'd like it to be
            pdef_->addSolutionPath (sol, true, 0.0);
    }

    si_->freeStates(xstates);

    // Return true if any solution was found.
    return sol ? (addedNewSolution() ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::APPROXIMATE_SOLUTION) : base::PlannerStatus::TIMEOUT;
}

ompl::geometric::PRM::Vertex ompl::geometric::PRM::addMilestone(base::State *state)
{
    graphMutex_.lock();
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);
    graphMutex_.unlock();

    // Which milestones will we attempt to connect to?
    if (!connectionStrategy_)
        throw Exception(name_, "No connection strategy!");

    const std::vector<Vertex>& neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if ((boost::same_component(m, n, disjointSets_) || connectionFilter_(m, n)))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[m], stateProperty_[n]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const double weight = distanceFunction(m, n);
                const unsigned int id = maxEdgeID_++;
                const Graph::edge_property_type properties(weight, id);

                graphMutex_.lock();
                boost::add_edge(m, n, properties, g_);
                uniteComponents(n, m);
                graphMutex_.unlock();
            }
        }

    nn_->add(m);
    return m;
}

void ompl::geometric::PRM::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

ompl::base::PathPtr ompl::geometric::PRM::constructSolution(const Vertex start, const Vertex goal) const
{
    PathGeometric *p = new PathGeometric(si_);

    graphMutex_.lock();
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    boost::astar_search(g_, start,
            boost::bind(&PRM::distanceFunction, this, _1, goal),
            boost::predecessor_map(prev));
    graphMutex_.unlock();

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    return base::PathPtr(p);
}

void ompl::geometric::PRM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[startM_[i]]));

    for (size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[goalM_[i]]));

    // Adding edges and all other vertices simultaneously
    foreach(const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]),
                     base::PlannerDataVertex(stateProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]),
                     base::PlannerDataVertex(stateProperty_[v1]));
    }
}
