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
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::KPIECE1::KPIECE1(const base::SpaceInformationPtr &si)
  : base::Planner(si, "KPIECE1"), disc_([this](Motion *m) { freeMotion(m); })
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &KPIECE1::setRange, &KPIECE1::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &KPIECE1::setGoalBias, &KPIECE1::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("border_fraction", this, &KPIECE1::setBorderFraction, &KPIECE1::getBorderFraction,
                                  "0.:0.05:1.");
    Planner::declareParam<double>("failed_expansion_score_factor", this, &KPIECE1::setFailedExpansionCellScoreFactor,
                                  &KPIECE1::getFailedExpansionCellScoreFactor);
    Planner::declareParam<double>("min_valid_path_fraction", this, &KPIECE1::setMinValidPathFraction,
                                  &KPIECE1::getMinValidPathFraction);
}

ompl::geometric::KPIECE1::~KPIECE1() = default;

void ompl::geometric::KPIECE1::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    if (failedExpansionScoreFactor_ < std::numeric_limits<double>::epsilon() || failedExpansionScoreFactor_ > 1.0)
        throw Exception("Failed expansion cell score factor must be in the range (0,1]");
    if (minValidPathFraction_ < std::numeric_limits<double>::epsilon() || minValidPathFraction_ > 1.0)
        throw Exception("The minimum valid path fraction must be in the range (0,1]");

    disc_.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::KPIECE1::clear()
{
    Planner::clear();
    sampler_.reset();
    disc_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::KPIECE1::freeMotion(Motion *motion)
{
    if (motion->state != nullptr)
        si_->freeState(motion->state);
    delete motion;
}

ompl::base::PlannerStatus ompl::geometric::KPIECE1::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    Discretization<Motion>::Coord xcoord(projectionEvaluator_->getDimension());

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        projectionEvaluator_->computeCoordinates(motion->state, xcoord);
        disc_.addMotion(motion, xcoord, 1.0);
    }

    if (disc_.getMotionCount() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                disc_.getMotionCount());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        disc_.countIteration();

        /* Decide on a state to expand from */
        Motion *existing = nullptr;
        Discretization<Motion>::Cell *ecell = nullptr;
        disc_.selectMotion(existing, ecell);
        assert(existing);

        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(xstate);
        else
            sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);

        std::pair<base::State *, double> fail(xstate, 0.0);
        bool keep = si_->checkMotion(existing->state, xstate, fail);
        if (!keep && fail.second > minValidPathFraction_)
            keep = true;

        if (keep)
        {
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;

            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            projectionEvaluator_->computeCoordinates(motion->state, xcoord);
            disc_.addMotion(motion, xcoord, dist);  // this will also update the discretization heaps as needed, so no
                                                    // call to updateCell() is needed

            if (solv)
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
        else
            ecell->data->score *= failedExpansionScoreFactor_;
        disc_.updateCell(ecell);
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

    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(),
                disc_.getMotionCount(), disc_.getCellCount(), disc_.getGrid().countInternal(),
                disc_.getGrid().countExternal());

    return {solved, approximate};
}

void ompl::geometric::KPIECE1::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    disc_.getPlannerData(data, 0, true, lastGoalMotion_);
}
