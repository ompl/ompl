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

/* Author: Ryan Luna */

#include "ompl/geometric/planners/est/EST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::EST::EST(const base::SpaceInformationPtr &si) : base::Planner(si, "EST")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &EST::setRange, &EST::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &EST::setGoalBias, &EST::getGoalBias, "0.:.05:1.");
}

ompl::geometric::EST::~EST()
{
    freeMemory();
}

void ompl::geometric::EST::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // Make the neighborhood radius smaller than sampling range to keep probabilities relatively high for rejection
    // sampling
    nbrhoodRadius_ = maxDistance_ / 3.0;

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

void ompl::geometric::EST::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    motions_.clear();
    pdf_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::EST::freeMemory()
{
    for (auto &motion : motions_)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
}

ompl::base::PlannerStatus ompl::geometric::EST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    std::vector<Motion *> neighbors;

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);

        nn_->nearestR(motion, nbrhoodRadius_, neighbors);
        addMotion(motion, neighbors);
    }

    if (motions_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), motions_.size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();
    auto *xmotion = new Motion();

    while (!ptc)
    {
        // Select a state to expand from
        Motion *existing = pdf_.sample(rng_.uniform01());
        assert(existing);

        // Sample random state in the neighborhood (with goal biasing)
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(xstate);

            // Compute neighborhood of candidate motion
            xmotion->state = xstate;
            nn_->nearestR(xmotion, nbrhoodRadius_, neighbors);
        }
        else
        {
            // Sample a state in the neighborhood
            if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
                continue;

            // Compute neighborhood of candidate state
            xmotion->state = xstate;
            nn_->nearestR(xmotion, nbrhoodRadius_, neighbors);

            // reject state with probability proportional to neighborhood density
            if (!neighbors.empty() )
            {
                double p = 1.0 - (1.0 / neighbors.size());
                if (rng_.uniform01() < p)
                    continue;
            }
        }

        // Is motion good?
        if (si_->checkMotion(existing->state, xstate))
        {
            // create a motion
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;

            // add it to everything
            addMotion(motion, neighbors);

            // done?
            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
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

        // construct the solution path
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    delete xmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), motions_.size());

    return {solved, approximate};
}

void ompl::geometric::EST::addMotion(Motion *motion, const std::vector<Motion *> &neighbors)
{
    // Updating neighborhood size counts
    for (auto neighbor : neighbors)
    {
        PDF<Motion *>::Element *elem = neighbor->element;
        double w = pdf_.getWeight(elem);
        pdf_.update(elem, w / (w + 1.));
    }

    // now add new motion to the data structures
    motion->element = pdf_.add(motion, 1. / (neighbors.size() + 1.));  // +1 for self
    motions_.push_back(motion);
    nn_->add(motion);
}

void ompl::geometric::EST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto motion : motions_)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
