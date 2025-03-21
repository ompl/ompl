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

#include "ompl/geometric/planners/est/ProjEST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::ProjEST::ProjEST(const base::SpaceInformationPtr &si) : base::Planner(si, "ProjEST")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &ProjEST::setRange, &ProjEST::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &ProjEST::setGoalBias, &ProjEST::getGoalBias, "0.:.05:1.");
}

ompl::geometric::ProjEST::~ProjEST()
{
    freeMemory();
}

void ompl::geometric::ProjEST::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    tree_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::ProjEST::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    tree_.grid.clear();
    tree_.size = 0;
    pdf_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::ProjEST::freeMemory()
{
    for (const auto &it : tree_.grid)
    {
        for (auto &motion : it.second->data.motions_)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::ProjEST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        addMotion(motion);
    }

    if (tree_.grid.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree_.size);

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* Decide on a state to expand from */
        Motion *existing = selectMotion();
        assert(existing);

        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(xstate);
        else if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
            continue;

        if (si_->checkMotion(existing->state, xstate))
        {
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;

            addMotion(motion);
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

    OMPL_INFORM("%s: Created %u states in %u cells", getName().c_str(), tree_.size, tree_.grid.size());

    return {solved, approximate};
}

ompl::geometric::ProjEST::Motion *ompl::geometric::ProjEST::selectMotion()
{
    GridCell *cell = pdf_.sample(rng_.uniform01());
    return (cell != nullptr) && !cell->data.empty() ? cell->data[rng_.uniformInt(0, cell->data.size() - 1)] : nullptr;
}

void ompl::geometric::ProjEST::addMotion(Motion *motion)
{
    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    GridCell *cell = tree_.grid.getCell(coord);
    if (cell != nullptr)
    {
        cell->data.push_back(motion);
        pdf_.update(cell->data.elem_, 1.0 / cell->data.size());
    }
    else
    {
        cell = tree_.grid.createCell(coord);
        cell->data.push_back(motion);
        tree_.grid.add(cell);
        cell->data.elem_ = pdf_.add(cell, 1.0);
    }
    tree_.size++;
}

void ompl::geometric::ProjEST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<MotionInfo> motionInfo;
    tree_.grid.getContent(motionInfo);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &m : motionInfo)
        for (auto &motion : m.motions_)
        {
            if (motion->parent == nullptr)
                data.addStartVertex(base::PlannerDataVertex(motion->state));
            else
                data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
        }
}
