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

#include "ompl/geometric/planners/prm/BasicPRM.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

/** \brief Maximum number of sampling attempts to find a valid state,
    without checking whether the allowed time elapsed. This value
    should not really be changed. */
static const unsigned int FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK = 2;


ompl::geometric::BasicPRM::BasicPRM(const base::SpaceInformationPtr &si) :
    base::Planner(si, "PRM"),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    edgeIDProperty_(boost::get(boost::edge_index, g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    maxEdgeID_(0)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    connectionStrategy_ = KStrategy<Vertex>(10, nn_);
    connectionFilter_ = boost::lambda::constant(true);
}

void ompl::geometric::BasicPRM::setup(void)
{
    Planner::setup();
    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Vertex>());
    nn_->setDistanceFunction(boost::bind(&BasicPRM::distanceFunction, this, _1, _2));
}

void ompl::geometric::BasicPRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    startM_.clear();
    goalM_.clear();
}

void ompl::geometric::BasicPRM::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    startM_.clear();
    goalM_.clear();
    maxEdgeID_ = 0;
}

void ompl::geometric::BasicPRM::freeMemory(void)
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

void ompl::geometric::BasicPRM::growRoadmap(double growTime)
{
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
            } while (attempts < FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK && !found);
        }
        // add it as a milestone
        if (found)
            addMilestone(si_->cloneState(workState));
    }
    si_->freeState(workState);
}

void ompl::geometric::BasicPRM::growRoadmap(const std::vector<Vertex> &start,
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
            } while (attempts < FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK && !found);
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

bool ompl::geometric::BasicPRM::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, std::pair<Vertex, Vertex> *endpoints)
{
    base::Goal *g = pdef_->getGoal().get();
    foreach (Vertex start, starts)
        foreach (Vertex goal, goals)
        {

            if (boost::same_component(start, goal, disjointSets_) &&
                g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                if (endpoints)
                {
                    endpoints->first = start;
                    endpoints->second = goal;
                }
                return true;
            }
        }
    return false;
}

bool ompl::geometric::BasicPRM::solve(const base::PlannerTerminationCondition &ptc)
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

    msg_.inform("Starting with %u states", nrStartStates);

    base::State *xstate = si_->allocState();
    std::pair<Vertex, Vertex> solEndpoints;

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
            constructSolution(solEndpoints.first, solEndpoints.second);
            break;
        }
        // othewise, spend some time building a roadmap
        else
        {
            // if it is worth looking at other goal regions, plan for part of the time
            if (goal->maxSampleCount() > goalM_.size())
                growRoadmap(startM_, goalM_, ptc + base::timedPlannerTerminationCondition(0.1), xstate);
            // otherwise, just go ahead and build the roadmap
            else
                growRoadmap(startM_, goalM_, ptc, xstate);
            // if a solution has been found, construct it
            if (haveSolution(startM_, goalM_, &solEndpoints))
            {
                constructSolution(solEndpoints.first, solEndpoints.second);
                break;
            }
        }
    }
    si_->freeState(xstate);

    msg_.inform("Created %u states", boost::num_vertices(g_) - nrStartStates);

    return goal->isAchieved();
}

ompl::geometric::BasicPRM::Vertex ompl::geometric::BasicPRM::addMilestone(base::State *state)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    // Which milestones will we attempt to connect to?
    if (!connectionStrategy_)
        throw Exception(name_, "No connection strategy!");

    const std::vector<Vertex>& neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if ((boost::same_component(m, n, disjointSets_)
             || connectionFilter_(m, n))
                && si_->checkMotion(stateProperty_[m], stateProperty_[n]))
        {
            const double weight = distanceFunction(m, n);
            const unsigned int id = maxEdgeID_++;
            const Graph::edge_property_type properties(weight, id);
            boost::add_edge(m, n, properties, g_);
            uniteComponents(n, m);
        }

    nn_->add(m);
    return m;
}

void ompl::geometric::BasicPRM::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

void ompl::geometric::BasicPRM::constructSolution(const Vertex start, const Vertex goal)
{
    PathGeometric *p = new PathGeometric(si_);

    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    boost::astar_search(g_, start,
            boost::bind(&BasicPRM::distanceFunction, this, _1, goal),
            boost::predecessor_map(prev));

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->states.push_back(si_->cloneState(stateProperty_[pos]));
    p->states.push_back(si_->cloneState(stateProperty_[start]));
    p->reverse();

    pdef_->getGoal()->setSolutionPath(base::PathPtr(p));
}

void ompl::geometric::BasicPRM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    foreach(const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.recordEdge(stateProperty_[v1], stateProperty_[v2]);
    }
}
