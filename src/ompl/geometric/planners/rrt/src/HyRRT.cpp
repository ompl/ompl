/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, University of Santa Cruz Hybrid Systems Laboratory
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

/* Author: Beverly Xu */

#include <list>

#include "ompl/geometric/planners/rrt/HyRRT.h"
#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

namespace base = ompl::base;
namespace tools = ompl::tools;

using namespace std::chrono;

ompl::geometric::HyRRT::HyRRT(const base::SpaceInformationPtr &si_) : base::Planner(si_, "HyRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;
}

ompl::geometric::HyRRT::~HyRRT()
{
}

void ompl::geometric::HyRRT::initTree(void)
{
    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->hybridTime->push_back(std::pair<double, int>(0.0, 0));
        // Add start motion to the tree
        nn_->add(motion);
    }
}

void ompl::geometric::HyRRT::randomSample(Motion *randomMotion)
{
    sampler_ = si_->allocStateSampler();
    // Replace later with the ompl sampler, for now leave as custom
    sampler_->sampleUniform(randomMotion->state);
}

base::PlannerStatus ompl::geometric::HyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // Initialization
    // Make sure the planner is configured correctly
    // Ensures that there is at least one input state and a goal object specified
    checkValidity();
    checkMandatoryParametersSet();

    // Define control space
    control::RealVectorControlSpace *controlSpace_ = new control::RealVectorControlSpace(
        si_->getStateSpace(),
        minJumpInputValue_.size() > minFlowInputValue_.size() ? minJumpInputValue_.size() : minFlowInputValue_.size());

    int numInputs =
        minFlowInputValue_.size() > minJumpInputValue_.size() ? minFlowInputValue_.size() : minJumpInputValue_.size();

    // Vectors for storing flow and jump inputs
    std::vector<double> flowInputs;
    std::vector<double> jumpInputs;

    initTree();

    // Allocate memory for a random motion
    auto *randomMotion = new Motion(si_);

    // Main Planning Loop
    // Periodically check if the termination condition is met
    // If it is, terminate planning
    while (!ptc())
    {
    nextIteration:
        randomSample(randomMotion);  // Randomly sample a state from the planning space

        auto *solution = new Motion(si_);

        // Generate random maximum flow time
        double random = rand();
        double randomFlowTimeMax = random / RAND_MAX * tM_;
        double tFlow = 0;  // Tracking variable for the amount of flow time used in a given continuous simulation step

        bool collision = false;  // Set collision to false initially

        // Sample and instantiate parent vertices and states in solutionPairs
        auto *parentMotion = nn_->nearest(randomMotion);
        base::State *previousState = si_->allocState();
        si_->copyState(previousState, parentMotion->state);
        auto *collisionParentMotion = nn_->nearest(randomMotion);

        // Initialize hybrid times for solution pair
        std::pair<double, int> hybridTimeInitial = parentMotion->hybridTime->back();

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
        std::vector<std::pair<double, int>> *hybridTimes = new std::vector<std::pair<double, int>>();

        // Simulate in either the jump or flow regime
        if (!priority)
        {  // Flow
            // Randomly sample the flow inputs
            flowInputs = sampleFlowInputs_();
            control::Control *flowInput = controlSpace_->allocControl();
            for (int i = 0; i < flowInputs.size(); i++)
                flowInput->as<control::RealVectorControlSpace::ControlType>()->values[i] = flowInputs[i];

            while (tFlow < randomFlowTimeMax && flowSet_(parentMotion))
            {
                tFlow += flowStepDuration_;
                hybridTimes->push_back(
                    std::pair<double, int>(tFlow + hybridTimeInitial.first, hybridTimeInitial.second));

                // Find new state with continuous simulation
                base::State *intermediateState = si_->allocState();
                intermediateState =
                    this->continuousSimulator_(flowInputs, previousState, flowStepDuration_, intermediateState);

                // Add new intermediate state to solutionPair
                intermediateStates->push_back(intermediateState);

                // Collision Checking
                double ts = hybridTimeInitial.first;
                double tf = hybridTimeInitial.first + tFlow;

                // Create motion to add to tree
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, intermediateState);
                motion->parent = parentMotion;
                motion->solutionPair = intermediateStates;  // Set the new motion solutionPair
                motion->hybridTime = hybridTimes;
                for (int i = 0; i < intermediateStates->size(); i++)
                    motion->inputs->push_back(flowInput);

                // Discard state if it is in the unsafe set
                if (unsafeSet_(motion))
                    goto nextIteration;

                double *collisionTime = new double(-1.0);
                collision = collisionChecker_(motion, jumpSet_, ts, tf, intermediateState, collisionTime);

                if (*collisionTime != -1.0)
                    hybridTimes->back().first = *collisionTime;

                // State has passed all tests so update parent, solutionPair, and temporary states
                si_->copyState(previousState, intermediateState);

                // Add motion to tree or handle collision/goal
                bool inGoalSet =
                    distanceFunc_(previousState, pdef_->getGoal()->as<base::GoalState>()->getState()) <= tolerance_;

                // If maximum flow time has been reached, a collision has occured, or a solution has been found, exit
                // the loop
                if (tFlow >= randomFlowTimeMax || collision || inGoalSet)
                {
                    hybridTimeInitial.first = hybridTimes->back().first;

                    if (inGoalSet)
                        solution = motion;
                    else if (collision)
                    {
                        collisionParentMotion = motion;
                        priority = true;  // If collision has occurred, continue to jump regime
                    }
                    else
                        nn_->add(motion);
                    break;
                }
            }
        }

        if (priority)
        {  // Jump
            // Randomly sample the jump inputs
            jumpInputs = sampleJumpInputs_();
            control::Control *jumpInput = controlSpace_->allocControl();

            for (int i = 0; i < jumpInputs.size(); i++)
            {
                jumpInput->as<control::RealVectorControlSpace::ControlType>()->values[i] = jumpInputs[i];
            }

            // Instantiate and find new state with discrete simulator
            base::State *newState = si_->allocState();
            newState = this->discreteSimulator_(previousState, jumpInputs, newState);

            // Create motion to add to tree
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, newState);
            motion->parent = collisionParentMotion;
            motion->inputs->push_back(jumpInput);
            motion->hybridTime->push_back(
                std::pair<double, int>(hybridTimeInitial.first, hybridTimeInitial.second + 1));

            // If generated state is in the unsafe set, then continue on to the next iteration
            if (unsafeSet_(motion))
                goto nextIteration;

            // Add motions to tree, and free up memory allocated to newState
            nn_->add(motion);
            nn_->add(collisionParentMotion);
        }

        // If state is within goal set, construct path
        if (distanceFunc_(previousState, pdef_->getGoal()->as<base::GoalState>()->getState()) <= tolerance_)
            return constructSolution(solution);
    }
}

base::PlannerStatus ompl::geometric::HyRRT::constructSolution(Motion *last_motion)
{
    vector<Motion *> trajectory;
    nn_->list(trajectory);
    std::vector<Motion *> mpath;

    double finalDistance = distanceFunc_(trajectory.back()->state, pdef_->getGoal()->as<base::GoalState>()->getState());
    Motion *solution = last_motion;

    int pathSize = 0;

    // Construct the path from the goal to the start by following the parent pointers
    while (solution != nullptr)
    {
        mpath.push_back(solution);
        if (solution->solutionPair != nullptr)  // A jump motion does not contain an solutionPair
            pathSize += solution->solutionPair->size();
        solution = solution->parent;
    }

    // Associate inputs and hybrid time with states of each solution pair
    int numInputs =
        minFlowInputValue_.size() > minJumpInputValue_.size() ? minFlowInputValue_.size() : minJumpInputValue_.size();
    ompl::base::RealVectorStateSpace *associateStateSpace = new ompl::base::RealVectorStateSpace(0);
    for (int i = 0; i < si_->getStateSpace()->getDimension() + numInputs + 2; i++)
    {
        associateStateSpace->addDimension(0, 1);  // Arbitrary values for initialization
    }

    ompl::base::StateSpacePtr space(associateStateSpace);

    // Construct a space information instance for this state space
    ompl::base::SpaceInformationPtr associatedSi(new ompl::base::SpaceInformation(space));

    associatedSi->setup();

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(associatedSi));

    // Create a new path object to store the solution path
    auto path(std::make_shared<PathGeometric>(associatedSi));

    // Add the states to the path in reverse order (from start to goal)
    for (int i = mpath.size() - 1; i >= 0; --i)
    {
        // Append all intermediate states to the path, including starting state,
        // excluding end vertex
        if (mpath[i]->solutionPair != nullptr)
        {  // A jump motion does not contain an solutionPair
            for (int k = 0; k < mpath[i]->solutionPair->size(); k++)
            {
                base::State *associatedState = associatedSi->allocState();

                // Add in state values
                if (k == mpath[i]->solutionPair->size() - 1)
                {
                    associatedState = mpath[i]->state;
                }
                else
                {
                    for (int j = 0; j < si_->getStateSpace()->getDimension(); j++)
                    {
                        associatedState->as<base::RealVectorStateSpace::StateType>()->values[j] =
                            mpath[i]->solutionPair->at(k)->as<base::RealVectorStateSpace::StateType>()->values[j];
                    }
                }

                // Add in input values
                for (int j = 0; j < numInputs; j++)
                {
                    associatedState->as<base::RealVectorStateSpace::StateType>()
                        ->values[si_->getStateSpace()->getDimension() + j] =
                        mpath[i]->inputs->at(k)->as<control::RealVectorControlSpace::ControlType>()->values[j];
                }

                // Add in hybrid time
                associatedState->as<base::RealVectorStateSpace::StateType>()
                    ->values[si_->getStateSpace()->getDimension() + numInputs] = mpath[i]->hybridTime->at(k).first;
                associatedState->as<base::RealVectorStateSpace::StateType>()
                    ->values[si_->getStateSpace()->getDimension() + numInputs + 1] = mpath[i]->hybridTime->at(k).second;

                path->append(associatedState);
            }
        }
        else
        {  // If a normal jump motion
            base::State *associatedState = associatedSi->allocState();

            // Add in state values
            for (int j = 0; j < si_->getStateSpace()->getDimension(); j++)
            {
                associatedState->as<base::RealVectorStateSpace::StateType>()->values[j] =
                    mpath[i]->state->as<base::RealVectorStateSpace::StateType>()->values[j];
            }

            // If not starting state
            if (i != mpath.size() - 1)
            {
                // Add in input values
                for (int j = 0; j < numInputs; j++)
                {
                    associatedState->as<base::RealVectorStateSpace::StateType>()
                        ->values[si_->getStateSpace()->getDimension() + j] =
                        mpath[i]->inputs->at(0)->as<control::RealVectorControlSpace::ControlType>()->values[j];
                }
            }

            // Add in hybrid time
            associatedState->as<base::RealVectorStateSpace::StateType>()
                ->values[si_->getStateSpace()->getDimension() + numInputs] = mpath[i]->hybridTime->back().first;
            associatedState->as<base::RealVectorStateSpace::StateType>()
                ->values[si_->getStateSpace()->getDimension() + numInputs + 1] = mpath[i]->hybridTime->back().second;

            path->append(associatedState);
        }
    }

    pdef_->addSolutionPath(path, finalDistance > 0.0, finalDistance, getName());

    // Return a status indicating that an exact solution has been found
    if (finalDistance > 0.0)
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    else
        return base::PlannerStatus::EXACT_SOLUTION;
}

void ompl::geometric::HyRRT::clear()
{
    Planner::clear();
    // clear the data structures here
}

// optional, if additional setup/configuration is needed, the setup() method can
// be implemented
void ompl::geometric::HyRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return ompl::geometric::HyRRT::distanceFunc_(a->state, b->state); });
}

void ompl::geometric::HyRRT::freeMemory(void)
{
}

void ompl::geometric::HyRRT::getPlannerData(base::PlannerData &data) const
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
