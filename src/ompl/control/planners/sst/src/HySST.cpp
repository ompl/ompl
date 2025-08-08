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

/* Authors: Beverly Xu */
/* Adapted from: ompl/geometric/planners/src/SST.cpp by  Zakary Littlefield of Rutgers the State University of New Jersey, New Brunswick */

#include "ompl/control/planners/sst/HySST.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

namespace base = ompl::base;
namespace tools = ompl::tools;
namespace control = ompl::control;

ompl::control::HySST::HySST(const control::SpaceInformationPtr &si_) : base::Planner(si_, "HySST")
{
    specs_.approximateSolutions = true;
    siC_ = si_.get();
    prevSolution_.clear();

    addPlannerProgressProperty("best cost REAL", [this]
                               { return std::to_string(this->prevSolutionCost_.value()); });
}

ompl::control::HySST::~HySST() {}

void ompl::control::HySST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return ompl::control::HySST::distanceFunc_(a->state, b->state); });
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    { return ompl::control::HySST::distanceFunc_(a->state, b->state); });

    if (pdef_ && pdef_->hasOptimizationObjective())
    {
        opt_ = pdef_->getOptimizationObjective();
        if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
            dynamic_cast<base::MinimaxObjective *>(opt_.get()))
            OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                      "functions w.r.t. state and control. This optimization objective will result in undefined "
                      "behavior",
                      getName().c_str());
        costFunc_ = [this](Motion *motion) -> base::Cost
        {
            return opt_->motionCost(motion->parent->state, motion->state);
        };
    }
    else
    {   // if no optimization objective set, assume we want to minimize hybrid time
        OMPL_WARN("%s: No optimization object set. Using hybrid time", getName().c_str());
        costFunc_ = [this](Motion *motion) -> base::Cost
        {
            return base::Cost(ompl::base::HybridStateSpace::getStateTime(motion->state) - ompl::base::HybridStateSpace::getStateTime(motion->parent->state) + ompl::base::HybridStateSpace::getStateJumps(motion->state) - ompl::base::HybridStateSpace::getStateJumps(motion->parent->state));
        };
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }
    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::control::HySST::clear()
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

void ompl::control::HySST::freeMemory()
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

ompl::control::HySST::Motion *ompl::control::HySST::selectNode(ompl::control::HySST::Motion *sample)
{
    std::vector<Motion *> ret; // List of all nodes within the selection radius
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret); // Find the nearest nodes within the selection radius of the random sample

    for (auto &i : ret) // Find the active node with the best cost within the selection radius
    {
        if (!i->inactive_ && i->accCost_.value() < bestCost.value())
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr) // However, if there are no active nodes within the selection radius, select the next nearest node
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);                                       // sample the k nearest nodes to the random sample into ret
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++) // Find the active node with the best cost
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5; // If none found, increase the number of nearest nodes to sample
        }
    }
    return selected;
}

ompl::control::HySST::Witness *ompl::control::HySST::findClosestWitness(ompl::control::HySST::Motion *node)
{
    auto *closest = new Witness(siC_);

    if (witnesses_->size() > 0)
        closest = static_cast<Witness *>(witnesses_->nearest(node));

    if (distanceFunc_(closest->state, node->state) > pruningRadius_ || witnesses_->size() == 0) // If the closest witness is outside the pruning radius or if there are no witnesses yet, return a new witness at the same point as the node.
    {
        closest->linkRep(node);
        si_->copyState(closest->state, node->state);
        witnesses_->add(closest);
    }
    return closest;
}

std::vector<ompl::control::HySST::Motion *> ompl::control::HySST::extend(Motion *parentMotion)
{
    control::Control *compoundControl = siC_->allocControl();
    siC_->allocControlSampler()->sample(compoundControl);

    // Generate random maximum flow time
    double random = rand();
    double randomFlowTimeMax = random / RAND_MAX * tM_;

    double tFlow = 0; // Tracking variable for the amount of flow time used in a given continuous simulation step
    bool collision = false; // Set collision to false initially

    // Choose whether to begin growing the tree in the flow or jump regime
    bool in_jump = jumpSet_(parentMotion);
    bool in_flow = flowSet_(parentMotion);
    bool priority = in_jump && in_flow ? random / RAND_MAX > 0.5 : in_jump; // If both are true, there is an equal chance of being in flow or jump set.

    // Sample and instantiate parent vertices and states in edges
    base::State *previousState = si_->allocState();
    si_->copyState(previousState, parentMotion->state);
    auto *collisionParentMotion = parentMotion;

    // Allocate memory for the new edge
    std::vector<base::State *> *intermediateStates = new std::vector<base::State *>;

    // Simulate in either the jump or flow regime
    if (!priority) // Flow
    {
        while (tFlow < randomFlowTimeMax && flowSet_(parentMotion))
        {
            tFlow += flowStepDuration_;

            // Find new state with continuous simulation
            base::State *intermediateState = si_->allocState();
            intermediateState = this->continuousSimulator_(getFlowControl(compoundControl), previousState, flowStepDuration_, intermediateState);
            ompl::base::HybridStateSpace::setStateTime(intermediateState, ompl::base::HybridStateSpace::getStateTime(previousState) + flowStepDuration_);
            ompl::base::HybridStateSpace::setStateJumps(intermediateState, ompl::base::HybridStateSpace::getStateJumps(previousState));

            // Add new intermediate state to edge
            intermediateStates->push_back(intermediateState);

            // Create motion to add to tree
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, intermediateState);
            motion->parent = parentMotion;
            motion->solutionPair = intermediateStates; // Set the new motion solutionPair
            motion->control = compoundControl;

            // Return nullptr if in unsafe set and exit function
            if (unsafeSet_(motion))
                return std::vector<Motion *>(); // Return empty vector

            double *collisionTime = new double(-1.0);
            collision = collisionChecker_(motion, jumpSet_, intermediateState, collisionTime);
            
            if (*collisionTime != -1.0){
                ompl::base::HybridStateSpace::setStateTime(motion->state, *collisionTime);
                ompl::base::HybridStateSpace::setStateTime(intermediateState, *collisionTime);
            }

            // State has passed all tests so update parent, edge, and temporary states
            si_->copyState(previousState, intermediateState);

            // Calculate distance to goal to check if solution has been found
            bool inGoalSet = pdef_->getGoal()->isSatisfied(intermediateState);

            // If maximum flow time has been reached, a collision has occured, or a solution has been found, exit the loop
            if (tFlow >= randomFlowTimeMax || collision || inGoalSet)
            {
                if (inGoalSet)
                    return std::vector<Motion *>{motion};
                else if (collision)
                {
                    collisionParentMotion = motion;
                    priority = true; // If collision has occurred, continue to jump regime
                }
                else {
                    return std::vector<Motion *>{motion}; // Return the motion in vector form
                }
                break;
            }
        }
    }

    if (priority)
    { // Jump
        // Instantiate and find new state with discrete simulator
        base::State *newState = si_->allocState();

        newState = this->discreteSimulator_(previousState, getJumpControl(compoundControl), newState);

        // Create motion to add to tree
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, newState);
        motion->parent = collisionParentMotion;
        motion->control = compoundControl;
        ompl::base::HybridStateSpace::setStateTime(motion->state, ompl::base::HybridStateSpace::getStateTime(previousState));
        ompl::base::HybridStateSpace::setStateJumps(motion->state, ompl::base::HybridStateSpace::getStateJumps(previousState) + 1);

        // Return nullptr if in unsafe set and exit function
        if (unsafeSet_(motion))
            return std::vector<Motion *>(); // Return empty vector

        // Add motions to tree, and free up memory allocated to newState
        collisionParentMotion->numChildren_++;

        if (tFlow > 0)  // If coming from flow propagation
            return std::vector<Motion *>{motion, collisionParentMotion};
        else
            return std::vector<Motion *>{motion}; 
    }
    return std::vector<Motion *>();
}

void ompl::control::HySST::randomSample(Motion *randomMotion)
{
    sampler_->sampleUniform(randomMotion->state);
}

ompl::base::PlannerStatus ompl::control::HySST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    this->setContinuousSimulator(continuousSimulator);
    checkMandatoryParametersSet();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
        ompl::base::HybridStateSpace::setStateTime(motion->state, 0.0);
        ompl::base::HybridStateSpace::setStateJumps(motion->state, 0);
        motion->accCost_ = base::Cost(0.0); // Initialize the accumulated cost to the identity cost
        findClosestWitness(motion);         // Set representatives for the witness set
    }

    if (!sampler_)
        sampler_ = siC_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;

    int solutions = 0;

    while (!ptc)
    {
        // sample random state
        randomSample(rmotion);

        // find closest state in the tree
        Motion *nmotion = selectNode(rmotion);

        std::vector<Motion *> dMotion = {new Motion(siC_)};

        dMotion = extend(nmotion);

        if (dMotion.size() == 0) // If extension failed, continue to next iteration
            continue;

        si_->copyState(rstate, dMotion[0]->state); // copy the new state to the random state pointer. First value of dMotion vector will always be the newest state, even if a collision occurs

        base::Cost incCost = costFunc_(dMotion[0]);    // Compute incremental cost
        base::Cost cost = base::Cost(nmotion->accCost_.value() + incCost.value()); // Combine total cost

        auto *collisionParentMotion = new Motion(siC_);
        if (dMotion.size() > 1) // If collision occured during extension
        {
            collisionParentMotion = dMotion[1];
            collisionParentMotion->accCost_ = base::Cost(nmotion->accCost_.value() + costFunc_(dMotion[1]).value());
            cost = base::Cost(cost.value() + costFunc_(dMotion[1]).value());
        }

        Witness *closestWitness = findClosestWitness(rmotion);            // Find closest witness

        if (closestWitness->rep_ == rmotion || cost.value() < closestWitness->rep_->accCost_.value()) // If the newly propagated state is a child of the new representative of the witness (previously had no rep) or it dominates the old representative's cost
        {
            Motion *oldRep = closestWitness->rep_; // Set a copy of the old representative
            /* create a motion copy  of the newly propagated state */
            auto *motion = new Motion(siC_);

            if (dMotion.size() > 1) // If collision occured during extension
            {
                nn_->add(collisionParentMotion);
            }

            motion = dMotion[0];
            motion->accCost_ = cost;

            nmotion->numChildren_++;
            closestWitness->linkRep(motion); // Create new edge and set the new node as the representative

            nn_->add(motion); // Add new node to tree
            bool solved = pdef_->getGoal()->isSatisfied(motion->state, &dist_);
            
            if (solved && motion->accCost_.value() < prevSolutionCost_.value()) // If the new state is a solution and it has a lower cost than the previous solution
            {
                approxdif = dist_;
                solution = motion;

                prevSolution_.clear();
                Motion *solTrav = solution; // Traverse the solution and save the states in prevSolution_
                while (solTrav != nullptr)
                {
                    prevSolution_.push_back(solTrav);
                    solTrav = solTrav->parent;
                }

                prevSolutionCost_ = solution->accCost_; 

                OMPL_INFORM("Solution found with cost of %f", prevSolutionCost_.value());          
                solutions++;
                if(solutions >= batchSize_)
                    break;
            }
            if (solution == nullptr && dist_ < approxdif) // If no solution found and distance to goal of this new state is closer than before (because no guarantee of probabilistic completeness). Also where approximate solutions are filled.
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

            if (oldRep != rmotion) // If the representative has changed (prune)
            {
                oldRep->inactive_ = true; // Mark the node as inactive
                while (oldRep->inactive_ && oldRep->numChildren_ == 0) // While the current node is inactive and is a leaf, remove it (non-leaf nodes have been marked inactive)
                {
                    Motion *oldRepParent = oldRep->parent;
                    oldRep = oldRepParent;
                    oldRep->numChildren_--;

                    if (oldRep->numChildren_ == 0)
                        oldRep->inactive_ = true; // Now that its only child has been removed, this node is inactive as well
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;

    if (solution == nullptr) // If closest state to goal is outside goal set
        solution = approxsol;
    else
        solved = true;
    
    if (approxdif != 0)    // Approximate if not exactly the goal state
        approximate = true;

    if (solution != nullptr)    // If any state has been successfully propagated
    {
        // Set the solution path
        constructSolution(solution);
    }

    return {solved, approximate};
}

ompl::base::PlannerStatus ompl::control::HySST::constructSolution(Motion *last_motion)
{
    std::vector<Motion *> trajectory;
    nn_->list(trajectory);
    std::vector<Motion *> mpath;

    double finalDistance = 0;
    pdef_->getGoal()->isSatisfied(trajectory.back()->state, &finalDistance);

    Motion *solution = last_motion;

    int pathSize = 0;

    // Construct the path from the goal to the start by following the parent pointers
    while (solution != nullptr)
    {
        mpath.push_back(solution);
        if (solution->solutionPair != nullptr)              // A jump motion does not contain an edge
            pathSize += solution->solutionPair->size() + 1; // +1 for the end state
        solution = solution->parent;
    }

    // Create a new path object to store the solution path
    auto path(std::make_shared<control::PathControl>(si_));

    // Reserve space for the path states
    path->getStates().reserve(pathSize);

    // Add the states to the path in reverse order (from start to goal)
    for (int i = mpath.size() - 1; i >= 0; --i)
    {
        // Append all intermediate states to the path, including starting state,
        // excluding end vertex
        if (mpath[i]->solutionPair != nullptr)
        { // A jump motion does not contain an edge
            for (unsigned int j = 0; j < mpath[i]->solutionPair->size(); j++)
            {
                if (i == 0 && j == 0)   // Starting state has no control
                { 
                    path->append(mpath[i]->solutionPair->at(j));
                    continue;
                }
                path->append(mpath[i]->solutionPair->at(j), mpath[i]->control, siC_->getPropagationStepSize()); // Need to make a new motion to append to trajectory matrix
            }
        }
        else
        { // If a jump motion
            if (i == 0)
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

void ompl::control::HySST::getPlannerData(base::PlannerData &data) const
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