/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Darie Roman
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

    pathToGoal = false;
    regrowing_ = false;

    Planner::declareParam<double>("range", this, &DRRT::setRange, &DRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &DRRT::setGoalBias, &DRRT::getGoalBias, "0.:.05:1.");
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
    lastGoalMotion_ = nullptr;
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
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing, and waypoint biasing if regrowing tree) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else if ()
        else
            sampler_->sampleUniform(rstate);

        /* find the closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

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

                    nmotion = motion;
                }
            }
            else
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);

                nmotion = motion;
            }

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
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

        // Update the solution path
        path_.clear();
        while (solution != nullptr)
        {
            path_.push_back(solution);
            solution->onPath = true;
            solution = solution->parent;
        }

        // Set the solution path in the ProblemDefinition
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = path_.size() - 1; i >= 0; --i)
            path->append(path_[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
        pathToGoal_ = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::DRRT::updateEnvironment()
{
    // Flag that says if a path to the goal has been invalidated
    bool pathInvalidated = false;

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return;
    }

    // Queue of directly invalidated motions
    std::vector<Motion *> invalidatedMotions;
    // Queue of motions to remove recursively
    std::queue<Motion *, std::deque<Motion *>> motionRemoveQueue;

    // Loop through motions in tree, add all directly invalidated ones to queue for removal
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto &motion : motions)
    {
        if (si_->checkMotion(motion->parent->state, motion->state) == false)
        {
            invalidatedMotions.push_back(motion);
            motionRemoveQueue.push(motion);
        }
    }

    // Remove motions by propagating leaf-ward from invalidated motions
    while (motionRemoveQueue.empty() == false)
    {
        // If this motion is a goal, path is destroyed and tree begins regrowing process
        if (pathToGoal_ && motionRemoveQueue.front()->inGoal == true)
            pathToGoal_ = false;
            regrowing_ = true;
            pathInvalidated = true;

        // Add all children of this motion to the removal queue
        addChildrenToList(&motionRemoveQueue, motionRemoveQueue.front());

        // Remove this motion from its parent
        removeFromParent(motionRemoveQueue.front());

        // Free the motion's state
        si_->freeState(motionRemoveQueue.front()->state);

        // Delete the motion's pointer
        delete motionRemoveQueue.front();

        // Remove the motion from the removal queue
        motionRemoveQueue.pop();
    }

    // Update waypoints if a path to the goal was invalidated
    if (pathInvalidated)
    {
        waypoints_.clear();
        // Look for the first motion that was on the now invalidated path
        for (auto &invalidatedMotion : invalidatedMotions)
        {
            if (invalidatedMotion->onPath)
            {
                // Add all waypoints starting here, and ending at the tree root
                Motion *motion = invalidatedMotion;
                while (motion->parent != nullptr)
                {
                    if (si_->checkMotion(motion->parent->state, motion->state))
                        waypoints_.push_back(motion);
                    motion = motion->parent;
                }
                break;
            }
        }
    }

    // reset onPath flag of motions that used to be on the now invalidated path
    for (auto &motion : path_)
    {
        motion->onPath = false;
    }
    // clear path vector
    path_.clear();

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

void ompl::geometric::DRRT::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

void ompl::geometric::DRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

