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

#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>
#include <cassert>

void ompl::geometric::KPIECE1::setup(void)
{
    Planner::setup();
    checkProjectionEvaluator(this, projectionEvaluator_);
    checkMotionLength(this, maxDistance_);

    if (badScoreFactor_ < std::numeric_limits<double>::epsilon() || badScoreFactor_ > 1.0)
        throw Exception("Bad cell score factor must be in the range (0,1]");
    if (goodScoreFactor_ < std::numeric_limits<double>::epsilon() || goodScoreFactor_ > 1.0)
        throw Exception("Good cell score factor must be in the range (0,1]");
    if (minValidPathFraction_ < std::numeric_limits<double>::epsilon() || minValidPathFraction_ > 1.0)
        throw Exception("The minimum valid path fraction must be in the range (0,1]");

    disc_.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::KPIECE1::clear(void)
{
    Planner::clear();
    sampler_.reset();
    disc_.clear();
}

void ompl::geometric::KPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
        si_->freeState(motion->state);
    delete motion;
}

bool ompl::geometric::KPIECE1::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalRegion           *goal_r = dynamic_cast<base::GoalRegion*>(goal);
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if (!goal)
    {
        msg_.error("Goal undefined");
        return false;
    }

    Discretization<Motion>::Coord xcoord;

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        projectionEvaluator_->computeCoordinates(motion->state, xcoord);
        disc_.addMotion(motion, xcoord, 1.0);
    }

    if (disc_.getTreeData().grid.size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    if (!sampler_)
        sampler_ = si_->allocManifoldStateSampler();

    msg_.inform("Starting with %u states", disc_.getTreeData().size);

    Motion *solution    = NULL;
    Motion *approxsol   = NULL;
    double  approxdif   = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();

    while (ptc() == false)
    {
        disc_.countIteration();

        /* Decide on a state to expand from */
        Motion                       *existing = NULL;
        Discretization<Motion>::Cell *ecell    = NULL;
        disc_.selectMotion(existing, ecell);
        assert(existing);

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(xstate);
        else
            sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);

        std::pair<base::State*, double> fail(xstate, 0.0);
        bool keep = si_->checkMotion(existing->state, xstate, fail);
        if (!keep && fail.second > minValidPathFraction_)
            keep = true;

        if (keep)
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;

            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            projectionEvaluator_->computeCoordinates(motion->state, xcoord);
            disc_.addMotion(motion, xcoord, dist);

            if (solved)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
            ecell->data->score *= goodScoreFactor_;
        }
        else
            ecell->data->score *= badScoreFactor_;

        disc_.updateCell(ecell);
    }

    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
           for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->states.push_back(si_->cloneState(mpath[i]->state));
        goal->setDifference(approxdif);
        goal->setSolutionPath(base::PathPtr(path), approximate);

        if (approximate)
            msg_.warn("Found approximate solution");
    }

    si_->freeState(xstate);

    msg_.inform("Created %u states in %u cells (%u internal + %u external)", disc_.getMotionCount(), disc_.getCellCount(),
                disc_.getTreeData().grid.countInternal(), disc_.getTreeData().grid.countExternal());

    return goal->isAchieved();
}

void ompl::geometric::KPIECE1::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    disc_.getPlannerData(data, 0);
}
