/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Darie Roman
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

/* Authors: Darie Roman */

#include "ompl/geometric/planners/rrt/DRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::DRRT::DRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "DRRTintermediate" : "DRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    pathToGoal_ = false;
    regrowing_ = false;

    Planner::declareParam<double>("range", this, &DRRT::setRange, &DRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("start_bias", this, &DRRT::setStartBias, &DRRT::getStartBias, "0.:.05:1.");
    Planner::declareParam<double>("waypoint_bias", this, &DRRT::setWaypointBias, &DRRT::getWaypointBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &DRRT::setIntermediateStates, &DRRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::DRRT::~DRRT()
{
    freeMemory();
}

void ompl::geometric::DRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastStartMotion_ = nullptr;
}

void ompl::geometric::DRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::DRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::DRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if (pdef_->getStartStateCount() > 1)
    {
        OMPL_ERROR("%s: There is more than one start state!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // startState is where robot currently is, gets randomly sampled occasionally
    const base::State *startState = pdef_->getStartState(0);

    // Goal is where the robot wants to be, tree is rooted here
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Goal cannot be cast to GoalSampleableRegion", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    bool replanning = false;

    // Check if tree is non-empty, i.e., solve was called before on this planning instance
    std::vector<Motion *> currentMotions;
    nn_->list(currentMotions);
    if (!currentMotions.empty())
    {
        OMPL_DEBUG("Tree already exists.");

        // Check if the goal is the same, meaning that we should replan with previous tree

        // New goal state from problem definition
        base::State *newGoalState = si_->allocState();
        goal_s->sampleGoal(newGoalState);

        // Current goal state from existing tree
        base::State *currentGoalState = currentMotions[0]->state;

        // The goal is the same as before if the two goal states are equal
        if (si_->equalStates(currentGoalState, newGoalState))
        {
            // Prune existing tree to account for changes in the environment
            pruneTree();
            nn_->list(currentMotions);

            // Reset inStart if robot start position changed
            for (auto &m : currentMotions)
            {
                if (m->inStart && !si_->equalStates(m->state, startState))
                    m->inStart = false;
            }

            replanning = true;
        }
        // The goal is different, start from scratch
        else
            clear();
    }

    if (!replanning)
    {
        OMPL_DEBUG("Starting planning from scratch.");
        // Add the goal motion to the tree
        base::State *rootState = si_->allocState();
        goal_s->sampleGoal(rootState);
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, rootState);
        nn_->add(motion);
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    Motion *solution = nullptr;
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        double p = rng_.uniform01();
        // Chance to sample start state
        if (p < startBias_)
            si_->copyState(rstate, startState);
        // If tree is regrowing, chance to sample a random waypoint
        else if (regrowing_ && waypoints_.size() > 1 && p > startBias_ && p < startBias_ + waypointBias_)
        {
            // Sample a random waypoint into rstate
            int waypointSampleIdx = rng_.uniformInt(0, waypoints_.size() - 1);
            Motion *sampledWaypoint = waypoints_[waypointSampleIdx];
            si_->copyState(rstate, sampledWaypoint->state);

            // Delete waypoint once it has been sampled to avoid resampling
            waypoints_.erase(waypoints_.begin() + waypointSampleIdx);
            delete sampledWaypoint;
        }
        else
            sampler_->sampleUniform(rstate);

        // Find the closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        // Find state to add
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, dstate))
        {
            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);

                    nmotion = motion;
                }
            }
            else
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);
                motion->parent->children.push_back(motion);

                nmotion = motion;
            }

            if (si_->equalStates(nmotion->state, startState))
            {
                solution = nmotion;
                nmotion->inStart = true;
                break;
            }
        }
    }

    bool solved = false;

    if (solution != nullptr)
    {
        lastStartMotion_ = solution;

        // Get the motion along the solution path
        std::vector<Motion *> solutionPathReversed;
        while (solution != nullptr)
        {
            solutionPathReversed.push_back(solution);
            solution = solution->parent;
        }

        // Set the solution path in the ProblemDefinition
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = solutionPathReversed.size() - 1; i >= 0; --i)
            path->append(solutionPathReversed[i]->state);
        pdef_->addSolutionPath(path, false, -1.0, getName());
        solved = true;
        pathToGoal_ = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Tree has %u states", getName().c_str(), nn_->size());

    return {solved, false};
}

void ompl::geometric::DRRT::pruneTree()
{

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return;
    }

    std::vector<Motion *> motions;
    nn_->list(motions);
    std::set<Motion *> directlyInvalidatedMotions;

    // Loop through motions in tree, add all root invalidated ones to vector
    for (auto &motion : motions)
    {
        if (motion->parent != nullptr && !si_->checkMotion(motion->parent->state, motion->state))
            directlyInvalidatedMotions.insert(motion);
    }

    // Add children of all directly invalidated motions to invalidated motions by propagating from roots inward
    std::set<Motion *> invalidatedMotions(directlyInvalidatedMotions.begin(),
                                             directlyInvalidatedMotions.end());
    for (auto &directlyInvalidatedMotion : directlyInvalidatedMotions)
    {
        std::vector<Motion *> invalidatedChildren;
        getAllChildren(directlyInvalidatedMotion, invalidatedChildren, directlyInvalidatedMotions);
        if (!invalidatedChildren.empty())
            std::copy(invalidatedChildren.begin(), invalidatedChildren.end(),
                      std::inserter(invalidatedMotions, invalidatedMotions.end()));
    }

    // Update waypoints if a path to the goal has become invalidated
    for (auto &invalidatedMotion : invalidatedMotions)
    {
        if (pathToGoal_ && invalidatedMotion->inStart)
        {
            pathToGoal_ = false;
            regrowing_ = true;

            // delete current waypoints
            while (!waypoints_.empty())
            {
                Motion *waypoint = *waypoints_.begin();
                waypoints_.erase(waypoints_.begin());
                delete waypoint;
            }

            // Start motion is invalidated, follow it to obstacle and add waypoints along the way
            Motion *pathMotion = invalidatedMotion;
            while (pathMotion->parent != nullptr && si_->checkMotion(pathMotion->parent->state, pathMotion->state))
            {
                pathMotion = pathMotion->parent;
                auto *newMotion = new Motion(si_);
                si_->copyState(newMotion->state, pathMotion->state);
                waypoints_.push_back(newMotion);
            }
            break;
        }
    }

    OMPL_INFORM("Pruning %i nodes.", invalidatedMotions.size());

    // Remove all invalidated motions from the tree and delete them
    while (!invalidatedMotions.empty())
    {
        Motion *invalidatedMotion = *invalidatedMotions.begin();

        // Remove from tree
        nn_->remove(invalidatedMotion);

        // remove from set
        invalidatedMotions.erase(invalidatedMotion);

        // Remove from parent if possible
        if (invalidatedMotion->parent->state != nullptr && !invalidatedMotion->parent->children.empty())
            removeFromParent(invalidatedMotion);

        // Free the motion's state
        if (invalidatedMotion->state != nullptr)
            si_->freeState(invalidatedMotion->state);

        // Delete the motion's pointer
        delete invalidatedMotion;
    }

    OMPL_INFORM("Tree now has %u nodes.", nn_->size());

}

void ompl::geometric::DRRT::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::DRRT::getAllChildren(const Motion *m,  std::vector<Motion *> &cs,
                                           const std::set<Motion *> &directlyInvalidatedMotions)
{
    // Loop over children of motion
    for (auto &c : m->children)
    {
        cs.push_back(c);
        getAllChildren(c, cs, directlyInvalidatedMotions);
    }
}

void ompl::geometric::DRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastStartMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastStartMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

