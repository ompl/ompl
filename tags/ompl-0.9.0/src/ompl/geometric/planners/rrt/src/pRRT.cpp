/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/geometric/planners/rrt/pRRT.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <boost/thread/thread.hpp>
#include <limits>

void ompl::geometric::pRRT::setup(void)
{
    Planner::setup();
    checkMotionLength(this, maxDistance_);

    if (!nn_)
        nn_.reset(new NearestNeighborsSqrtApprox<Motion*>());
    nn_->setDistanceFunction(boost::bind(&pRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::pRRT::clear(void)
{
    Planner::clear();
    samplerArray_.clear();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::pRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::pRRT::threadSolve(unsigned int tid, const base::PlannerTerminationCondition &ptc, SolutionInfo *sol)
{
    pis_.checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    RNG                         rng;

    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (sol->solution == NULL && ptc() == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            samplerArray_[tid]->sampleUniform(rstate);

        /* find closest state in the tree */
        nnLock_.lock();
        Motion *nmotion = nn_->nearest(rmotion);
        nnLock_.unlock();
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateManifold()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nnLock_.lock();
            nn_->add(motion);
            nnLock_.unlock();

            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            if (solved)
            {
                sol->lock.lock();
                sol->approxdif = dist;
                sol->solution = motion;
                sol->lock.unlock();
                break;
            }
            if (dist < sol->approxdif)
            {
                sol->lock.lock();
                if (dist < sol->approxdif)
                {
                    sol->approxdif = dist;
                    sol->approxsol = motion;
                }
                sol->lock.unlock();
            }
        }
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;
}

bool ompl::geometric::pRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    base::GoalRegion *goal = dynamic_cast<base::GoalRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        msg_.error("Goal undefined");
        return false;
    }

    samplerArray_.resize(threadCount_);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    msg_.inform("Starting with %u states", nn_->size());

    SolutionInfo sol;
    sol.solution = NULL;
    sol.approxsol = NULL;
    sol.approxdif = std::numeric_limits<double>::infinity();

    std::vector<boost::thread*> th(threadCount_);
    for (unsigned int i = 0 ; i < threadCount_ ; ++i)
        th[i] = new boost::thread(boost::bind(&pRRT::threadSolve, this, i, ptc, &sol));
    for (unsigned int i = 0 ; i < threadCount_ ; ++i)
    {
        th[i]->join();
        delete th[i];
    }

    bool approximate = false;
    if (sol.solution == NULL)
    {
        sol.solution = sol.approxsol;
        approximate = true;
    }

    if (sol.solution != NULL)
    {
        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (sol.solution != NULL)
        {
            mpath.push_back(sol.solution);
            sol.solution = sol.solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
           for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->states.push_back(si_->cloneState(mpath[i]->state));

        goal->setDifference(sol.approxdif);
        goal->setSolutionPath(base::PathPtr(path), approximate);

        if (approximate)
            msg_.warn("Found approximate solution");
    }

    msg_.inform("Created %u states", nn_->size());

    return goal->isAchieved();
}

void ompl::geometric::pRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

void ompl::geometric::pRRT::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);
    threadCount_ = nthreads;
}
