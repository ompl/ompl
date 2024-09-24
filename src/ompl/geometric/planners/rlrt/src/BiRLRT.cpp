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

#include "ompl/geometric/planners/rlrt/BiRLRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::BiRLRT::BiRLRT(const base::SpaceInformationPtr &si) : base::Planner(si, "BiRLRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &BiRLRT::setRange, &BiRLRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("max_dist_near", this, &BiRLRT::setMaxDistanceNear, &BiRLRT::getMaxDistanceNear,
                                  "0.:0.1:10.");
    Planner::declareParam<bool>("keep_last_valid", this, &BiRLRT::setKeepLast, &BiRLRT::getKeepLast, "0,1");
}

ompl::geometric::BiRLRT::~BiRLRT()
{
    freeMemory();
}

void ompl::geometric::BiRLRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

void ompl::geometric::BiRLRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    if (range_ < 1e-4)
        sc.configurePlannerRange(range_);

    if (maxDistNear_ < 1e-4)
        maxDistNear_ = range_ / 20.0;  // make this pretty small
}

void ompl::geometric::BiRLRT::freeMemory()
{
    for (auto &motion : tStart_)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    for (auto &motion : tGoal_)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }

    tStart_.clear();
    tGoal_.clear();
}

/// Try to grow the tree randomly.  Return true if a new state was added
bool ompl::geometric::BiRLRT::growTreeRangeLimited(std::vector<Motion *> &tree, Motion *xmotion)
{
    assert(tree.size() > 0);

    // select a state from tree to expand from
    Motion *randomMotion = tree[rng_.uniformInt(0, tree.size() - 1)];

    // Sample a random direction.  Limit length of motion to range_, if necessary
    sampler_->sampleUniform(xmotion->state);
    double d = si_->distance(randomMotion->state, xmotion->state);
    if (d > range_)
        si_->getStateSpace()->interpolate(randomMotion->state, xmotion->state, range_ / d, xmotion->state);

    if (si_->checkMotion(randomMotion->state, xmotion->state))
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, xmotion->state);
        motion->parent = randomMotion;
        motion->root = randomMotion->root;

        tree.push_back(motion);
        return true;
    }

    return false;
}

/// Try to grow the tree randomly.  Return true if a new state was added
bool ompl::geometric::BiRLRT::growTreeKeepLast(std::vector<Motion *> &tree, Motion *xmotion,
                                               std::pair<ompl::base::State *, double> &lastValid)
{
    assert(tree.size() > 0);

    // select a state from tree to expand from
    Motion *randomMotion = tree[rng_.uniformInt(0, tree.size() - 1)];

    // Sample a random state.
    sampler_->sampleUniform(xmotion->state);

    lastValid.second = 0.0;
    bool valid = si_->checkMotion(randomMotion->state, xmotion->state, lastValid);
    if (valid || lastValid.second > 1e-3)
    {
        // create a new motion
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, valid ? xmotion->state : lastValid.first);
        motion->parent = randomMotion;
        motion->root = randomMotion->root;

        tree.push_back(motion);
        return true;
    }

    return false;
}

int ompl::geometric::BiRLRT::connectToTree(const Motion *motion, std::vector<Motion *> &tree)
{
    assert(tree.size() > 0);

    int checks = tree.size() > 1 ? ceil(log(tree.size())) : 1;

    for (int i = 0; i < checks; ++i)
    {
        // select a state from tree to expand from
        int randIndex = rng_.uniformInt(0, tree.size() - 1);
        Motion *randomMotion = tree[randIndex];

        if (si_->checkMotion(randomMotion->state, motion->state))
            return randIndex;
    }

    return -1;  // failed connection
}

ompl::base::PlannerStatus ompl::geometric::BiRLRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_.push_back(motion);
    }

    if (tStart_.size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_.size() + tGoal_.size()));
    if (keepLast_)
        OMPL_INFORM("%s: keeping last valid state", getName().c_str());
    else
        OMPL_INFORM("%s: tree is range limited", getName().c_str());

    std::vector<Motion *> *tree, *otherTree;
    tree = &tStart_;
    otherTree = &tGoal_;
    bool solved = false;

    auto xmotion = new Motion(si_);

    std::pair<ompl::base::State *, double> lastValid;
    lastValid.first = si_->allocState();

    while (ptc == false && solved == false)
    {
        if (tGoal_.size() == 0 || pis_.getSampledGoalsCount() < tGoal_.size() / 2)
        {
            const base::State *st = tGoal_.size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_.push_back(motion);
            }

            if (tGoal_.size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        bool expanded = keepLast_ ? growTreeKeepLast(*tree, xmotion, lastValid) : growTreeRangeLimited(*tree, xmotion);
        if (expanded)
        {
            int connectionIdx = connectToTree(tree->back(), *otherTree);
            if (connectionIdx >= 0)
            {
                // there is a solution path.  construct it.
                Motion *startMotion = tree == &tStart_ ? tree->back() : otherTree->at(connectionIdx);
                Motion *goalMotion = tree == &tStart_ ? otherTree->at(connectionIdx) : tree->back();

                // start and goal pair is not valid for this problem
                if (!goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
                    continue;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                PathGeometric *path = new PathGeometric(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (unsigned int i = 0; i < mpath2.size(); ++i)
                    path->append(mpath2[i]->state);

                pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
                solved = true;
            }
        }
        std::swap(tree, otherTree);
    }

    si_->freeState(xmotion->state);
    delete xmotion;
    si_->freeState(lastValid.first);

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_.size() + tGoal_.size(),
                tStart_.size(), tGoal_.size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::BiRLRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    for (auto &motion : tStart_)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }
    for (auto &motion : tGoal_)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
