/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of Santa Cruz Hybrid Systems Laboratory
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
 *   * Neither the name of the University of Santa Cruz nor the names of 
 *     its contributors may be used to endorse or promote products derived
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

/* Author: Beverly Xu */

#include "ompl/control/planners/rrt/HyRRT.h"
#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

namespace base = ompl::base;
namespace tools = ompl::tools;
namespace control = ompl::control;

using namespace std::chrono;

ompl::control::HyRRT::HyRRT(const control::SpaceInformationPtr &si_) : base::Planner(si_, "HyRRT")
{
    specs_.approximateSolutions = false;
    siC_ = si_.get();
}

ompl::control::HyRRT::~HyRRT() {
    freeMemory();
}

void ompl::control::HyRRT::initTree(void)
{
    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        motion->root = motion->state;
        ompl::base::HybridStateSpace::setStateTime(motion->state, 0.0);
        ompl::base::HybridStateSpace::setStateJumps(motion->state, 0);
        // Add start motion to the tree
        nn_->add(motion);
    }

    if (!sampler_)
        sampler_ = siC_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();
}

void ompl::control::HyRRT::randomSample(Motion *randomMotion)
{
    sampler_->sampleUniform(randomMotion->state);
}

base::PlannerStatus ompl::control::HyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // Initialization
    // Make sure the planner is configured correctly
    // Ensures that there is at least one input state and a goal object specified
    checkValidity();
    this->setContinuousSimulator(continuousSimulator);
    checkMandatoryParametersSet();
    initTree();

    // Main Planning Loop
    // Periodically check if the termination condition is met
    // If it is, terminate planning
    while (!ptc())
    {
    nextIteration:

        // Allocate memory for a random motion
        auto *randomMotion = new Motion(siC_);
        randomSample(randomMotion);  // Randomly sample a state from the planning space

        auto *solution = new Motion(siC_);

        // Generate random maximum flow time
        double random = rand();
        double randomFlowTimeMax = random / RAND_MAX * tM_;
        double tFlow = 0;  // Tracking variable for the amount of flow time used in a given continuous simulation step

        bool collision = false;  // Set collision to false initially

        // Sample and instantiate parent vertices and states in solutionPairs
        auto *parentMotion = nn_->nearest(randomMotion);
        base::State *previousState = si_->allocState();
        si_->copyState(previousState, parentMotion->state);
        auto *collisionParentMotion = parentMotion;

        // Choose whether to begin growing the tree in the flow or jump regime
        bool in_jump = jumpSet_(parentMotion);
        bool in_flow = flowSet_(parentMotion);
        bool priority = in_jump && in_flow ? random / RAND_MAX > 0.5 :
                                             in_jump;  // If both are true, equal chance of being in flow or jump set.

        // Allocate memory for the new solutionPair
        std::vector<base::State *> *intermediateStates = new std::vector<base::State *>;

        // Fill the solutionPair with the starting vertex
        base::State *parentState = si_->allocState();
        si_->copyState(parentState, previousState);

        control::Control *compoundControl = siC_->allocControl();
        siC_->allocControlSampler()->sample(compoundControl);

        // Simulate in either the jump or flow regime
        if (!priority)
        {  // Flow
            while (tFlow < randomFlowTimeMax && flowSet_(parentMotion))
            {
                tFlow += flowStepDuration_;

                // Find new state with continuous simulation
                base::State *intermediateState = si_->allocState();

                intermediateState = this->continuousSimulator_(getFlowControl(compoundControl), previousState,
                                                               flowStepDuration_, intermediateState);
                ompl::base::HybridStateSpace::setStateTime(
                    intermediateState, ompl::base::HybridStateSpace::getStateTime(previousState) + flowStepDuration_);
                ompl::base::HybridStateSpace::setStateJumps(intermediateState,
                                                            ompl::base::HybridStateSpace::getStateJumps(previousState));

                // Add new intermediate state to solutionPair
                intermediateStates->push_back(intermediateState);

                // Create motion to add to tree
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, intermediateState);
                motion->parent = parentMotion;
                motion->solutionPair = intermediateStates;  // Set the new motion solutionPair

                motion->control = compoundControl;

                // Discard state if it is in the unsafe set
                if (unsafeSet_(motion))
                    goto nextIteration;

                double *collisionTime = new double(-1.0);
                collision = collisionChecker_(motion, jumpSet_, intermediateState, collisionTime);

                if (*collisionTime != -1.0)
                {
                    ompl::base::HybridStateSpace::setStateTime(motion->state, *collisionTime);
                    ompl::base::HybridStateSpace::setStateTime(intermediateState, *collisionTime);
                }

                // State has passed all tests so update parent, solutionPair, and temporary states
                si_->copyState(previousState, intermediateState);

                // Add motion to tree or handle collision/goal
                bool inGoalSet = pdef_->getGoal()->isSatisfied(previousState, &dist_);

                // If maximum flow time has been reached, a collision has occured, or a solution has been found, exit
                // the loop
                if (tFlow >= randomFlowTimeMax || collision || inGoalSet)
                {
                    if (inGoalSet)
                    {
                        solution = motion;
                        nn_->add(solution);
                    }
                    else if (collision)
                    {
                        collisionParentMotion = motion;
                        priority = true;  // If collision has occurred, continue to jump regime
                    }
                    else
                    {
                        nn_->add(motion);
                    }
                    break;
                }
            }
        }

        if (priority)
        {  // Jump
            // Instantiate and find new state with discrete simulator
            base::State *newState = si_->allocState();
            newState = this->discreteSimulator_(previousState, getJumpControl(compoundControl), newState);

            // Create motion to add to tree
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, newState);

            motion->parent = collisionParentMotion;
            motion->control = compoundControl;
            ompl::base::HybridStateSpace::setStateTime(motion->state,
                                                       ompl::base::HybridStateSpace::getStateTime(previousState));
            ompl::base::HybridStateSpace::setStateJumps(motion->state,
                                                        ompl::base::HybridStateSpace::getStateJumps(previousState) + 1);

            // If generated state is in the unsafe set, then continue on to the next iteration
            if (unsafeSet_(motion))
                goto nextIteration;

            // State has passed all tests so update parent, solutionPair, and temporary states
            si_->copyState(previousState, newState);

            // Add motions to tree, and free up memory allocated to newState
            nn_->add(motion);

            // Add motion to tree or handle collision/goal
            bool inGoalSet = pdef_->getGoal()->isSatisfied(previousState, &dist_);

            if (inGoalSet)
            {
                solution = motion;
            }
        }

        // If state is within goal set, construct path
        if (pdef_->getGoal()->isSatisfied(previousState, &dist_))
        {
            vector<Motion *> trajectory;
            return constructSolution(solution);
        }
    }
    return base::PlannerStatus::UNKNOWN;
}
base::PlannerStatus ompl::control::HyRRT::constructSolution(Motion *last_motion)
{
    vector<Motion *> trajectory;
    nn_->list(trajectory);
    std::vector<Motion *> mpath;

    double finalDistance = dist_;
    Motion *solution = last_motion;

    int pathSize = 0;

    // Construct the path from the goal to the start by following the parent pointers
    while (solution != nullptr)
    {
        mpath.push_back(solution);
        if (solution->solutionPair != nullptr)               // A jump motion does not contain an edge
            pathSize += solution->solutionPair->size() + 1;  // +1 for the end state
        solution = solution->parent;
    }

    // Create a new path object to store the solution path
    auto path(std::make_shared<control::PathControl>(si_));

    // Add the states to the path in reverse order (from start to goal)
    for (std::size_t i = mpath.size() - 1; i < mpath.size(); --i)
    {
        // Append all intermediate states to the path, including starting state,
        // excluding end vertex
        if (mpath[i]->solutionPair != nullptr)
        {  // A jump motion does not contain an edge
            for (unsigned int j = 0; j < mpath[i]->solutionPair->size(); j++)
            {
                if (i == mpath.size() - 1 && j == 0)  // Starting state has no control
                {
                    path->append(mpath[i]->solutionPair->at(j));
                    continue;
                }
                path->append(mpath[i]->solutionPair->at(j), mpath[i]->control,
                             siC_->getPropagationStepSize());  // Need to make a new motion to append to trajectory
                                                               // matrix
            }
        }
        else
        {  // If a jump motion
            if (i == mpath.size() - 1)
                path->append(mpath[i]->state);
            else
                path->append(mpath[i]->state, mpath[i]->control, 0);
        }
    }

    // Add the solution path to the problem definition
    pdef_->addSolutionPath(path, finalDistance > 0.0, finalDistance, getName());
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    // Return a status indicating that an exact solution has been found
    if (finalDistance > 0.0)
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    else
        return base::PlannerStatus::EXACT_SOLUTION;
}

void ompl::control::HyRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::control::HyRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return ompl::control::HyRRT::distanceFunc_(a->state, b->state); });
}

void ompl::control::HyRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            if (motion->solutionPair) 
            {
                for (base::State *state : *motion->solutionPair)
                    si_->freeState(state);
            }
            delete motion;
        }
    }
}

void ompl::control::HyRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    vector<Motion *> motions;
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