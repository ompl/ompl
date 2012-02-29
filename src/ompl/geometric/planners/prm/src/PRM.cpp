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
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/PDF.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

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
    userSetConnectionStrategy_(false)
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
    startM_.clear();
    goalM_.clear();
}

void ompl::geometric::PRM::clear(void)
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    startM_.clear();
    goalM_.clear();
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
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<Vertex> empty;
    std::vector<base::State*> states(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(empty, empty, base::timedPlannerTerminationCondition(expandTime), states);
    si_->freeStates(states);
}

void ompl::geometric::PRM::expandRoadmap(const std::vector<Vertex> &starts,
                                         const std::vector<Vertex> &goals,
                                         const base::PlannerTerminationCondition &ptc,
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

                // add the vertext to the nearest neighbors data structure
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
            if (haveSolution(starts, goals))
                break;
        }
    }
}

void ompl::geometric::PRM::growRoadmap(double growTime)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    time::point endTime = time::now() + time::seconds(growTime);
    base::State *workState = si_->allocState();
    while (time::now() < endTime)
    {
        // search for a valid state
        bool found = false;
        while (!found && time::now() < endTime)
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
    si_->freeState(workState);
}

void ompl::geometric::PRM::growRoadmap(const std::vector<Vertex> &start,
                                       const std::vector<Vertex> &goal,
                                       const base::PlannerTerminationCondition &ptc,
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
        {
            addMilestone(si_->cloneState(workState));
            if (haveSolution(start, goal))
                break;
        }
    }
}

bool ompl::geometric::PRM::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, std::pair<Vertex, Vertex> *endpoints)
{
    base::Goal *g = pdef_->getGoal().get();
    foreach (Vertex start, starts)
        foreach (Vertex goal, goals)
        {

            if (boost::same_component(start, goal, disjointSets_) &&
                g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                bool sol = true;

                // if there is a maximum path length we wish to accept, we need to check the solution length
                if (g->getMaximumPathLength() < std::numeric_limits<double>::infinity())
                {
                    base::PathPtr p = constructSolution(start, goal);
                    double pl = p->length();
                    if (pl > g->getMaximumPathLength())
                    {
                        sol = false;
                        // record approximate solution
                        if (approxlen_ < 0.0 || approxlen_ > pl)
                        {
                            approxsol_ = p;
                            approxlen_ = pl;
                        }
                    }
                }

                if (sol)
                {
                    if (endpoints)
                    {
                        endpoints->first = start;
                        endpoints->second = goal;
                    }
                    return true;
                }
            }
        }
    return false;
}

bool ompl::geometric::PRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        msg_.error("Goal undefined or unknown type of goal");
        return false;
    }

    unsigned int nrStartStates = boost::num_vertices(g_);

    // add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    if (!goal->canSample())
    {
        msg_.error("Insufficient states in sampleable goal region");
        return false;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    approxsol_.reset();
    approxlen_ = -1.0;

    msg_.inform("Starting with %u states", nrStartStates);

    std::vector<base::State*> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    std::pair<Vertex, Vertex> solEndpoints;
    unsigned int steps = 0;
    bool addedSolution = false;

    while (ptc() == false)
    {
        // find at least one valid goal state
        if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
        {
            const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
            if (goalM_.empty())
            {
                msg_.error("Unable to find any valid goal states");
                break;
            }
        }

        // if there already is a solution, construct it
        if (haveSolution(startM_, goalM_, &solEndpoints))
        {
            goal->addSolutionPath(constructSolution(solEndpoints.first, solEndpoints.second));
            addedSolution = true;
            break;
        }
        // othewise, spend some time building a roadmap
        else
        {
            // maintain a 2:1 ration for growing:expansion of roadmap
            // call growRoadmap() 4 times for 0.1 seconds, for every call of expandRoadmap() for 0.2 seconds
            if (steps < 4)
            {
                growRoadmap(startM_, goalM_, base::PlannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(0.1)), xstates[0]);
                steps++;
            }
            else
            {
                expandRoadmap(startM_, goalM_, base::PlannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(0.2)), xstates);
                steps = 0;
            }

            // if a solution has been found, construct it
            if (haveSolution(startM_, goalM_, &solEndpoints))
            {
                goal->addSolutionPath(constructSolution(solEndpoints.first, solEndpoints.second));
                addedSolution = true;
                break;
            }
        }
    }
    si_->freeStates(xstates);

    msg_.inform("Created %u states", boost::num_vertices(g_) - nrStartStates);

    if (!addedSolution && approxsol_)
    {
        // the solution is exact, but not as short as we'd like it to be
        goal->addSolutionPath(approxsol_, true, 0.0);
        addedSolution = true;
    }

    return addedSolution;
}

ompl::geometric::PRM::Vertex ompl::geometric::PRM::addMilestone(base::State *state)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

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
                boost::add_edge(m, n, properties, g_);
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

ompl::base::PathPtr ompl::geometric::PRM::constructSolution(const Vertex start, const Vertex goal)
{
    PathGeometric *p = new PathGeometric(si_);

    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    boost::astar_search(g_, start,
            boost::bind(&PRM::distanceFunction, this, _1, goal),
            boost::predecessor_map(prev));

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

    foreach(const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.recordEdge(stateProperty_[v1], stateProperty_[v2]);
    }
}
