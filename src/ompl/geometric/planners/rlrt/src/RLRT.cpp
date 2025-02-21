/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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

/* Author: Ryan Luna */

#include "ompl/geometric/planners/rlrt/RLRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::RLRT::RLRT(const base::SpaceInformationPtr &si) : base::Planner(si, "RLRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("goal_bias", this, &RLRT::setGoalBias, &RLRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("range", this, &RLRT::setRange, &RLRT::getRange, "0.:1.:10000.");
    Planner::declareParam<bool>("keep_last_valid", this, &RLRT::setKeepLast, &RLRT::getKeepLast, "0,1");
}

ompl::geometric::RLRT::~RLRT()
{
    freeMemory();
}

void ompl::geometric::RLRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RLRT::setup()
{
    Planner::setup();

    if (range_ < 1e-4)
    {
        tools::SelfConfig sc(si_, getName());
        sc.configurePlannerRange(range_);
    }
}

void ompl::geometric::RLRT::freeMemory()
{
    for (auto &motion : motions_)
    {
        if (motion->state)
            si_->freeState(motion->state);
        delete motion;
    }

    motions_.clear();
}

ompl::base::PlannerStatus ompl::geometric::RLRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motions_.push_back(motion);
    }

    if (motions_.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), motions_.size());
    if (keepLast_)
        OMPL_INFORM("%s: keeping last valid state", getName().c_str());
    else
        OMPL_INFORM("%s: tree is range limited", getName().c_str());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    std::pair<ompl::base::State *, double> lastValid;
    lastValid.first = si_->allocState();

    while (ptc == false)
    {
        // Sample a state in the tree uniformly
        Motion *random = motions_[rng_.uniformInt(0, motions_.size() - 1)];

        // Sample a random state (with goal biasing)
        if (goal_s != nullptr && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        Motion *motion = nullptr;

        if (!keepLast_)  // range-limited random tree
        {
            // Truncate the distance of the randomly sampled state to range, if necessary
            double d = si_->distance(random->state, rstate);
            if (d > range_)
                si_->getStateSpace()->interpolate(random->state, rstate, range_ / d, rstate);

            // See if the path between them is valid
            if (si_->checkMotion(random->state, rstate))
            {
                // create a new motion
                motion = new Motion(si_);
                si_->copyState(motion->state, rstate);
                motion->parent = random;
                motions_.push_back(motion);
            }
        }
        else  // keep-last random tree
        {
            lastValid.second = 0.0;
            bool valid = si_->checkMotion(random->state, rstate, lastValid);
            if (valid || lastValid.second > 1e-3)
            {
                // create a new motion
                motion = new Motion(si_);
                si_->copyState(motion->state, valid ? rstate : lastValid.first);
                motion->parent = random;
                motions_.push_back(motion);
            }
        }

        // we added a state to the tree.  See if we're done.
        if (motion)
        {
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
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
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;
    si_->freeState(lastValid.first);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), motions_.size());

    return {solved, approximate};
}

void ompl::geometric::RLRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions_)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
