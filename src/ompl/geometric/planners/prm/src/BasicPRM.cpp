/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/prm/BasicPRM.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include <algorithm>
#include <queue>
#include <limits>

/** \brief Maximum number of sampling attempts to find a valid state,
    without checking whether the allowed time elapsed. This value
    should not really be changed. */
static const unsigned int FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK = 2;


void ompl::geometric::BasicPRM::setup(void)
{
    Planner::setup();
    if (!nn_)
        nn_.reset(new NearestNeighborsSqrtApprox<Milestone*>());
    nn_->setDistanceFunction(boost::bind(&BasicPRM::distanceFunction, this, _1, _2));
}

void ompl::geometric::BasicPRM::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    milestones_.clear();
    componentCount_ = 0;
    componentSizes_.clear();
    lastStart_ = NULL;
    lastGoal_ = NULL;
}

void ompl::geometric::BasicPRM::freeMemory(void)
{
    for (unsigned int i = 0 ; i < milestones_.size() ; ++i)
    {
        if (milestones_[i]->state)
            si_->freeState(milestones_[i]->state);
        delete milestones_[i];
    }
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

void ompl::geometric::BasicPRM::growRoadmap(const std::vector<Milestone*> &start,
                                            const std::vector<Milestone*> &goal,
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

bool ompl::geometric::BasicPRM::haveSolution(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal,
                                             std::pair<Milestone*, Milestone*> *endpoints)
{
    base::Goal *g = pdef_->getGoal().get();
    for (unsigned int i = 0 ; i < goal.size() ; ++i)
        for (unsigned int j = 0 ; j < start.size() ; ++j)
            if (goal[i]->component == start[j]->component && g->isStartGoalPairValid(goal[i]->state, start[j]->state))
            {
                if (endpoints)
                {
                    endpoints->first = start[j];
                    endpoints->second = goal[i];
                }
                return true;
            }
    return false;
}

namespace ompl
{
    // we grow a roadmap until the planner needs to stop or until a maximum amount of time has been reached
    static bool growRoadmapTerminationCondition(const base::PlannerTerminationCondition &ptc, const time::point &endTime)
    {
        return ptc() || time::now() >= endTime;
    }
}

bool ompl::geometric::BasicPRM::solve(const base::PlannerTerminationCondition &ptc)
{
    pis_.checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        msg_.error("Goal undefined or unknown type of goal");
        return false;
    }

    std::vector<Milestone*> startM;
    std::vector<Milestone*> goalM;

    // add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM.push_back(addMilestone(si_->cloneState(st)));

    if (startM.size() == 0)
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

    unsigned int nrStartStates = milestones_.size();
    msg_.inform("Starting with %u states", nrStartStates);

    base::State *xstate = si_->allocState();
    std::pair<Milestone*, Milestone*> solEndpoints;

    while (ptc() == false)
    {
        // find at least one valid goal state
        if (goal->maxSampleCount() > goalM.size())
        {
            const base::State *st = goalM.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
                goalM.push_back(addMilestone(si_->cloneState(st)));
            if (goalM.empty())
            {
                msg_.error("Unable to find any valid goal states");
                break;
            }
        }

        // if there already is a solution, construct it
        if (haveSolution(startM, goalM, &solEndpoints))
        {
            constructSolution(solEndpoints.first, solEndpoints.second);
            break;
        }
        // othewise, spend some time building a roadmap
        else
        {
            // if it is worth looking at other goal regions, plan for part of the time
            if (goal->maxSampleCount() > goalM.size())
                growRoadmap(startM, goalM, boost::bind(&growRoadmapTerminationCondition, ptc, time::now() + time::seconds(0.1)), xstate);
            // otherwise, just go ahead and build the roadmap
            else
                growRoadmap(startM, goalM, ptc, xstate);
            // if a solution has been found, construct it
            if (haveSolution(startM, goalM, &solEndpoints))
            {
                constructSolution(solEndpoints.first, solEndpoints.second);
                break;
            }
        }
    }
    si_->freeState(xstate);

    msg_.inform("Created %u states", milestones_.size() - nrStartStates);

    return goal->isAchieved();
}

void ompl::geometric::BasicPRM::nearestNeighbors(Milestone *milestone, std::vector<Milestone*> &nbh)
{
    nn_->nearestK(milestone, maxNearestNeighbors_, nbh);
}

ompl::geometric::BasicPRM::Milestone* ompl::geometric::BasicPRM::addMilestone(base::State *state)
{
    Milestone *m = new Milestone();
    m->state = state;
    m->component = componentCount_;
    componentSizes_[m->component] = 1;

    // connect to nearest neighbors
    std::vector<Milestone*> nbh;
    nearestNeighbors(m, nbh);

    for (unsigned int i = 0 ; i < nbh.size() ; ++i)
        if (si_->checkMotion(m->state, nbh[i]->state))
        {
            m->adjacent.push_back(nbh[i]);
            nbh[i]->adjacent.push_back(m);
            m->costs.push_back(si_->distance(m->state, nbh[i]->state));
            nbh[i]->costs.push_back(m->costs.back());
            uniteComponents(m, nbh[i]);
        }

    // if the new milestone was no absorbed in an existing component,
    // increase the number of components
    if (m->component == componentCount_)
        componentCount_++;
    m->index = milestones_.size();
    milestones_.push_back(m);
    nn_->add(m);
    return m;
}

void ompl::geometric::BasicPRM::uniteComponents(Milestone *m1, Milestone *m2)
{
    if (m1->component == m2->component)
        return;

    if (componentSizes_[m1->component] > componentSizes_[m2->component])
        std::swap(m1, m2);

    const unsigned long c = m2->component;
    componentSizes_[c] += componentSizes_[m1->component];
    componentSizes_.erase(m1->component);

    std::queue<Milestone*> q;
    q.push(m1);

    while (!q.empty())
    {
        Milestone *m = q.front();
        q.pop();
        if (m->component != c)
        {
            m->component = c;
            for (unsigned int i = 0 ; i < m->adjacent.size() ; ++i)
                if (m->adjacent[i]->component != c)
                    q.push(m->adjacent[i]);
        }
    }
}

void ompl::geometric::BasicPRM::reconstructLastSolution(void)
{
    if (lastStart_ && lastGoal_)
        constructSolution(lastStart_, lastGoal_);
    else
        msg_.warn("There is no solution to reconstruct");
}

void ompl::geometric::BasicPRM::constructSolution(const Milestone* start, const Milestone* goal)
{
    const unsigned int N = milestones_.size();
    std::vector<double> dist(N, std::numeric_limits<double>::infinity());
    std::vector<int>    prev(N, -1);
    std::vector<int>    seen(N, 0);

    dist[goal->index] = 0.0;
    for (unsigned int i = 0 ; i < N ; ++i)
    {
        double minDist = std::numeric_limits<double>::infinity();
        int index = -1;
        for (unsigned int j = 0 ; j < N ; ++j)
            if (seen[j] == 0 && dist[j] < minDist)
            {
                minDist = dist[j];
                index = j;
            }
        if (index < 0)
            break;
        seen[index] = 1;

        for (unsigned int j = 0 ; j < milestones_[index]->adjacent.size() ; ++j)
        {
            const unsigned int idx = milestones_[index]->adjacent[j]->index;
            double altDist = dist[index] + milestones_[index]->costs[j];
            if (altDist < dist[idx])
            {
                dist[idx] = altDist;
                prev[idx] = index;
            }
        }
    }
    if (prev[start->index] >= 0)
    {
        PathGeometric *p = new PathGeometric(si_);
        int pos = start->index;
        do
        {
            p->states.push_back(si_->cloneState(milestones_[pos]->state));
            pos = prev[pos];
        } while (pos >= 0);
        pdef_->getGoal()->setSolutionPath(base::PathPtr(p));
        lastStart_ = start;
        lastGoal_ = goal;
    }
    else
        throw Exception(name_, "Internal error in computing shortest path");
}

void ompl::geometric::BasicPRM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    for (unsigned int i = 0 ; i < milestones_.size() ; ++i)
        for (unsigned int j = 0 ; j < milestones_[i]->adjacent.size() ; ++j)
            data.recordEdge(milestones_[i]->state, milestones_[i]->adjacent[j]->state);
}
