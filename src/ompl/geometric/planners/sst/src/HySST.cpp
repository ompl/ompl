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
 *   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Beverly Xu */
/* Adapted from: ompl/geometric/planners/src/SST.cpp by  Zakary Littlefield of Rutgers the State University of New
 * Jersey, New Brunswick */

#include "ompl/geometric/planners/sst/HySST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <limits>
#include "ompl/base/goals/GoalState.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

ompl::geometric::HySST::HySST(const base::SpaceInformationPtr &si) : base::Planner(si, "HySST")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    prevSolution_.clear();

    addPlannerProgressProperty("best cost REAL", [this] { return std::to_string(this->prevSolutionCost_.value()); });
}

ompl::geometric::HySST::~HySST()
{
    freeMemory();
}

void ompl::geometric::HySST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return ompl::geometric::HySST::distanceFunc_(a->state, b->state); });
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    { return ompl::geometric::HySST::distanceFunc_(a->state, b->state); });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
                dynamic_cast<base::MinimaxObjective *>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                          "functions w.r.t. state and control. This optimization objective will result in undefined "
                          "behavior",
                          getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }
    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::geometric::HySST::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (witnesses_)
        witnesses_->clear();
    if (opt_)
        prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::geometric::HySST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
    if (witnesses_)
    {
        std::vector<Motion *> witnesses;
        witnesses_->list(witnesses);
        for (auto &witness : witnesses)
        {
            if (witness->state)
                si_->freeState(witness->state);
            delete witness;
        }
    }

    prevSolution_.clear();
}

ompl::geometric::HySST::Motion *ompl::geometric::HySST::selectNode(ompl::geometric::HySST::Motion *sample)
{
    std::vector<Motion *> ret;  // List of all nodes within the selection radius
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_,
                  ret);  // Find the nearest nodes within the selection radius of the random sample

    for (auto &i : ret)  // Find the active node with the best cost within the selection radius
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)  // However, if there are no active nodes within the selection radius, select the next
                              // nearest node
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);  // sample the k nearest nodes to the random sample into ret
            for (unsigned int i = 0; i < ret.size() && selected == nullptr;
                 i++)  // Find the active node with the best cost
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;  // If none found, increase the number of nearest nodes to sample
        }
    }
    return selected;
}

ompl::geometric::HySST::Witness *ompl::geometric::HySST::findClosestWitness(ompl::geometric::HySST::Motion *node)
{
    if (witnesses_->size() > 0)
    {
        auto *closest = static_cast<Witness *>(witnesses_->nearest(node));
        if (distanceFunc_(closest->state, node->state) > pruningRadius_)  // If the closest witness is outside the
                                                                          // pruning radius, return a new witness at the
                                                                          // same point as the node.
        {
            closest = new Witness(si_);
            closest->linkRep(node);
            si_->copyState(closest->state, node->state);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        auto *closest = new Witness(si_);
        closest->linkRep(node);
        si_->copyState(closest->state, node->state);
        witnesses_->add(closest);
        return closest;
    }
}

std::vector<ompl::geometric::HySST::Motion *> ompl::geometric::HySST::extend(Motion *parentMotion,
                                                                             base::Goal *goalState)
{
    // Initialize control inputs
    control::RealVectorControlSpace *controlSpace_ = new control::RealVectorControlSpace(
        si_->getStateSpace(),
        minJumpInputValue_.size() > minFlowInputValue_.size() ? minJumpInputValue_.size() : minFlowInputValue_.size());
    int numInputs =
        minFlowInputValue_.size() > minJumpInputValue_.size() ? minFlowInputValue_.size() : minJumpInputValue_.size();

    // Initialize hybrid times for solution pair
    std::pair<double, int> hybridTimeInitial = parentMotion->hybridTime->back();
    std::vector<std::pair<double, int>> *hybridTimes = new std::vector<std::pair<double, int>>();

    // Vectors for storing flow and jump inputs
    std::vector<double> flowInputs;
    std::vector<double> jumpInputs;

    // Generate random maximum flow time
    double random = rand();
    double randomFlowTimeMax = random / RAND_MAX * tM_;
    double tFlow = 0;  // Tracking variable for the amount of flow time used in a given continuous simulation step

    bool collision = false;  // Set collision to false initially

    // Choose whether to begin growing the tree in the flow or jump regime
    bool in_jump = jumpSet_(parentMotion);
    bool in_flow = flowSet_(parentMotion);
    bool priority = in_jump && in_flow ? random / RAND_MAX > 0.5 :
                                         in_jump;  // If both are true, equal chance of being in flow or jump set.

    // Sample and instantiate parent vertices and states in edges
    base::State *previousState = si_->allocState();
    si_->copyState(previousState, parentMotion->state);
    auto *collisionParentMotion = parentMotion;  // used to point to nn_->nearest(randomMotion);

    // Allocate memory for the new edge
    std::vector<base::State *> *intermediateStates = new std::vector<base::State *>;

    // Fill the edge with the starting vertex
    base::State *parentState = si_->allocState();
    si_->copyState(parentState, previousState);

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
            hybridTimes->push_back(std::pair<double, int>(tFlow + hybridTimeInitial.first, hybridTimeInitial.second));

            // Find new state with continuous simulation
            base::State *intermediateState = si_->allocState();
            intermediateState =
                this->continuousSimulator_(flowInputs, previousState, flowStepDuration_, intermediateState);

            // Add new intermediate state to edge
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

            // Return nullptr if in unsafe set and exit function
            if (unsafeSet_(motion))
                return std::vector<Motion *>();  // Return empty vector

            double *collisionTime = new double(-1.0);
            collision = collisionChecker_(motion, jumpSet_, ts, tf, intermediateState, collisionTime);

            if (*collisionTime != -1.0)
                hybridTimes->back().first = *collisionTime;

            // State has passed all tests so update parent, edge, and temporary states
            si_->copyState(previousState, intermediateState);

            // Add motion to tree or handle collision/goal
            dist_ = distanceFunc_(intermediateState, goalState->as<base::GoalState>()->getState());
            bool inGoalSet = dist_ <= tolerance_;

            // If maximum flow time has been reached, a collision has occured, or a solution has been found, exit the
            // loop
            if (tFlow >= randomFlowTimeMax || collision || inGoalSet)
            {
                hybridTimeInitial.first = hybridTimes->back().first;

                if (inGoalSet)
                {
                    return std::vector<Motion *>{motion};
                }
                else if (collision)
                {
                    collisionParentMotion = motion;
                    priority = true;  // If collision has occurred, continue to jump regime
                }
                else
                    return std::vector<Motion *>{motion};  // Return the motion in vector form
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
        motion->hybridTime->push_back(std::pair<double, int>(hybridTimeInitial.first, hybridTimeInitial.second + 1));

        // Return nullptr if in unsafe set and exit function
        if (unsafeSet_(motion))
            return std::vector<Motion *>();  // Return empty vector

        // Add motions to tree, and free up memory allocated to newState
        collisionParentMotion->numChildren_++;
        return std::vector<Motion *>{motion, collisionParentMotion};
    }
}

void ompl::geometric::HySST::randomSample(Motion *randomMotion)
{
    sampler_ = si_->allocStateSampler();
    // Replace later with the ompl sampler, for now leave as custom
    sampler_->sampleUniform(randomMotion->state);
}

ompl::base::PlannerStatus ompl::geometric::HySST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    checkMandatoryParametersSet();

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
        motion->hybridTime->push_back(std::pair<double, int>(0.0, 0));
        motion->accCost_ = opt_->identityCost();  // Initialize the accumulated cost to the identity cost
        findClosestWitness(motion);               // Set representatives for the witness set
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    unsigned solutions = 0;

    while (!ptc)
    {
        checkMandatoryParametersSet();
        // Define control space

        /* sample random state */
        randomSample(rmotion);

        /* find closest state in the tree */
        Motion *nmotion = selectNode(rmotion);

        std::vector<Motion *> dMotion = {new Motion(si_)};

        dMotion = extend(nmotion, goal);

        if (dMotion.size() == 0)  // If extension failed, continue to next iteration
            continue;

        si_->copyState(rstate,
                       dMotion[0]->state);  // copy the new state to the random state pointer. First value of dMotion
                                            // vector will always be the newest state, even if a collision occurs

        base::Cost incCost = opt_->motionCost(nmotion->state, rstate);     // Compute incremental cost
        base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);  // Combine total cost
        Witness *closestWitness = findClosestWitness(rmotion);             // Find closest witness

        if (closestWitness->rep_ == rmotion ||
            opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))  // If the newly propagated state is a child
                                                                           // of the new representative of the witness
                                                                           // (previously had no rep) or it dominates
                                                                           // the old representative's cost
        {
            Motion *oldRep = closestWitness->rep_;  // Set a copy of the old representative
            /* create a motion copy  of the newly propagated state */
            auto *motion = new Motion(si_);
            auto *collisionParentMotion = new Motion(si_);

            motion = dMotion[0];
            motion->accCost_ = cost;

            if (dMotion.size() > 1)  // If collision occured during extension
            {
                collisionParentMotion = dMotion[1];
                collisionParentMotion->accCost_ =
                    opt_->combineCosts(nmotion->accCost_, opt_->motionCost(nmotion->state, dMotion[1]->state));
                nn_->add(collisionParentMotion);
            }

            nmotion->numChildren_++;
            closestWitness->linkRep(motion);  // Create new edge and set the new node as the representative

            nn_->add(motion);  // Add new node to tree

            // dist_ is calculated during the call to extend()
            bool solv = dist_ <= tolerance_;
            if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))  // If the new state is a solution
                                                                                      // and it has a lower cost than
                                                                                      // the previous solution
            {
                approxdif = dist_;
                solution = motion;

                prevSolution_.clear();
                Motion *solTrav = solution;  // Traverse the solution and save the states in prevSolution_
                while (solTrav != nullptr)
                {
                    prevSolution_.push_back(solTrav);
                    solTrav = solTrav->parent;
                }
                prevSolutionCost_ = solution->accCost_;
                solutions++;
                if (solutions >= batchSize)
                    break;
            }
            if (solution == nullptr && dist_ < approxdif)  // If no solution found and distance to goal of this new
                                                           // state is closer than before (because no guarantee of
                                                           // probabilistic completeness). Also where approximate
                                                           // solutions are filled.
            {
                approxdif = dist_;
                approxsol = motion;

                prevSolution_.clear();
                Motion *solTrav = approxsol;
                while (solTrav != nullptr)
                {
                    prevSolution_.push_back(solTrav);
                    solTrav = solTrav->parent;
                }
                prevSolutionCost_ = motion->accCost_;
            }

            if (oldRep != rmotion)  // If the representative has changed (prune)
            {
                oldRep->inactive_ = true;                               // Mark the node as inactive
                while (oldRep->inactive_ && oldRep->numChildren_ == 0)  // While the current node is inactive and is a
                                                                        // leaf, remove it (non-leaf nodes have been
                                                                        // marked inactive)
                {
                    nn_->remove(oldRep);  // Remove from list of active nodes

                    if (oldRep->state)
                        si_->freeState(oldRep->state);

                    oldRep->state = nullptr;

                    Motion *oldRepParent = oldRep->parent;
                    delete oldRep;
                    oldRep = oldRepParent;
                    oldRep->numChildren_--;

                    if (oldRep->numChildren_ == 0)
                        oldRep->inactive_ =
                            true;  // Now that its only child has been removed, this node is inactive as well
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;

    if (solution == nullptr)  // If closest state to goal is outside goal set
        solution = approxsol;
    else
        solved = true;

    if (approxdif != 0)  // approximate if not exactly the goal state
        approximate = true;

    if (solution != nullptr)  // If any state has been successfully propagated
    {
        /* set the solution path */
        constructSolution(solution);
    }

    return {solved, approximate};
}

ompl::base::PlannerStatus ompl::geometric::HySST::constructSolution(Motion *last_motion)
{
    std::vector<Motion *> trajectory;
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

void ompl::geometric::HySST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    std::vector<Motion *> allMotions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
        if (motion->numChildren_ == 0)
            allMotions.push_back(motion);
    for (unsigned i = 0; i < allMotions.size(); i++)
        if (allMotions[i]->getParent() != nullptr)
            allMotions.push_back(allMotions[i]->getParent());

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]->state));

    for (auto &allMotion : allMotions)
    {
        if (allMotion->getParent() == nullptr)
            data.addStartVertex(base::PlannerDataVertex(allMotion->getState()));
        else
            data.addEdge(base::PlannerDataVertex(allMotion->getParent()->getState()),
                         base::PlannerDataVertex(allMotion->getState()));
    }
}
