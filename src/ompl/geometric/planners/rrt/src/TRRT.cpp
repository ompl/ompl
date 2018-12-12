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

/* Author: Dave Coleman, Ryan Luna */

#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <limits>

ompl::geometric::TRRT::TRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "TRRT")
{
    // Standard RRT Variables
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &TRRT::setRange, &TRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &TRRT::setGoalBias, &TRRT::getGoalBias, "0.:.05:1.");

    // TRRT Specific Variables
    frontierThreshold_ = 0.0;  // set in setup()
    setTempChangeFactor(0.1);  // how much to increase the temp each time
    costThreshold_ = base::Cost(std::numeric_limits<double>::infinity());
    initTemperature_ = 100;    // where the temperature starts out
    frontierNodeRatio_ = 0.1;  // 1/10, or 1 nonfrontier for every 10 frontier

    Planner::declareParam<double>("temp_change_factor", this, &TRRT::setTempChangeFactor, &TRRT::getTempChangeFactor,
                                  "0.:.1:1.");
    Planner::declareParam<double>("init_temperature", this, &TRRT::setInitTemperature, &TRRT::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold", this, &TRRT::setFrontierThreshold, &TRRT::getFrontierThreshold);
    Planner::declareParam<double>("frontier_node_ratio", this, &TRRT::setFrontierNodeRatio, &TRRT::getFrontierNodeRatio);
    Planner::declareParam<double>("cost_threshold", this, &TRRT::setCostThreshold, &TRRT::getCostThreshold);
}

ompl::geometric::TRRT::~TRRT()
{
    freeMemory();
}

void ompl::geometric::TRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nearestNeighbors_)
        nearestNeighbors_->clear();
    lastGoalMotion_ = nullptr;

    // Clear TRRT specific variables ---------------------------------------------------------
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1;  // init to 1 to prevent division by zero error
    if (opt_)
        bestCost_ = worstCost_ = opt_->identityCost();
}

void ompl::geometric::TRRT::setup()
{
    Planner::setup();
    tools::SelfConfig selfConfig(si_, getName());

    if (!pdef_ || !pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified.  Defaulting to mechanical work minimization.",
                    getName().c_str());
        opt_ = std::make_shared<base::MechanicalWorkOptimizationObjective>(si_);
    }
    else
        opt_ = pdef_->getOptimizationObjective();

    // Set maximum distance a new node can be from its nearest neighbor
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
        selfConfig.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    // Set the threshold that decides if a new node is a frontier node or non-frontier node
    if (frontierThreshold_ < std::numeric_limits<double>::epsilon())
    {
        frontierThreshold_ = si_->getMaximumExtent() * 0.01;
        OMPL_DEBUG("%s: Frontier threshold detected to be %lf", getName().c_str(), frontierThreshold_);
    }

    // Create the nearest neighbor function the first time setup is run
    if (!nearestNeighbors_)
        nearestNeighbors_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));

    // Set the distance function
    nearestNeighbors_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                           {
                                               return distanceFunction(a, b);
                                           });

    // Setup TRRT specific variables ---------------------------------------------------------
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1;  // init to 1 to prevent division by zero error
    bestCost_ = worstCost_ = opt_->identityCost();
}

void ompl::geometric::TRRT::freeMemory()
{
    // Delete all motions, states and the nearest neighbors data structure
    if (nearestNeighbors_)
    {
        std::vector<Motion *> motions;
        nearestNeighbors_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus
ompl::geometric::TRRT::solve(const base::PlannerTerminationCondition &plannerTerminationCondition)
{
    // Basic error checking
    checkValidity();

    // Goal information
    base::Goal *goal = pdef_->getGoal().get();
    auto *goalRegion = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Input States ---------------------------------------------------------------------------------

    // Loop through valid input states and add to tree
    while (const base::State *state = pis_.nextStart())
    {
        // Allocate memory for a new start state motion based on the "space-information"-size
        auto *motion = new Motion(si_);

        // Copy destination <= source
        si_->copyState(motion->state, state);

        // Set cost for this start state
        motion->cost = opt_->stateCost(motion->state);

        if (nearestNeighbors_->size() == 0)  // do not overwrite best/worst from previous call to solve
            worstCost_ = bestCost_ = motion->cost;

        // Add start motion to the tree
        nearestNeighbors_->add(motion);
    }

    // Check that input states exist
    if (nearestNeighbors_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Create state sampler if this is TRRT's first run
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // Debug
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                nearestNeighbors_->size());

    // Solver variables ------------------------------------------------------------------------------------

    // the final solution
    Motion *solution = nullptr;
    // the approximate solution, returned if no final solution found
    Motion *approxSolution = nullptr;
    // track the distance from goal to closest solution yet found
    double approxDifference = std::numeric_limits<double>::infinity();

    // distance between states - the intial state and the interpolated state (may be the same)
    double randMotionDistance;

    // Create random motion and a pointer (for optimization) to its state
    auto *randMotion = new Motion(si_);
    Motion *nearMotion;

    // STATES
    // The random state
    base::State *randState = randMotion->state;
    // The new state that is generated between states *to* and *from*
    base::State *interpolatedState = si_->allocState();  // Allocates "space information"-sized memory for a state
    // The chosen state btw rand_state and interpolated_state
    base::State *newState;

    // Begin sampling --------------------------------------------------------------------------------------
    while (plannerTerminationCondition() == false)
    {
        // I.

        // Sample random state (with goal biasing probability)
        if (goalRegion && rng_.uniform01() < goalBias_ && goalRegion->canSample())
        {
            // Bias sample towards goal
            goalRegion->sampleGoal(randState);
        }
        else
        {
            // Uniformly Sample
            sampler_->sampleUniform(randState);
        }

        // II.

        // Find closest state in the tree
        nearMotion = nearestNeighbors_->nearest(randMotion);

        // III.

        // Distance from near state q_n to a random state
        randMotionDistance = si_->distance(nearMotion->state, randState);

        // Check if the rand_state is too far away
        if (randMotionDistance > maxDistance_)
        {
            // Computes the state that lies at time t in [0, 1] on the segment that connects *from* state to *to* state.
            // The memory location of *state* is not required to be different from the memory of either *from* or *to*.
            si_->getStateSpace()->interpolate(nearMotion->state, randState, maxDistance_ / randMotionDistance,
                                              interpolatedState);

            // Update the distance between near and new with the interpolated_state
            randMotionDistance = si_->distance(nearMotion->state, interpolatedState);

            // Use the interpolated state as the new state
            newState = interpolatedState;
        }
        else  // Random state is close enough
            newState = randState;

        // IV.
        // this stage integrates collision detections in the presence of obstacles and checks for collisions
        if (!si_->checkMotion(nearMotion->state, newState))
            continue;  // try a new sample

        // Minimum Expansion Control
        // A possible side effect may appear when the tree expansion toward unexplored regions remains slow, and the
        // new nodes contribute only to refine already explored regions.
        if (!minExpansionControl(randMotionDistance))
            continue;  // give up on this one and try a new sample

        base::Cost childCost = opt_->stateCost(newState);

        // Only add this motion to the tree if the transition test accepts it
        if (!transitionTest(opt_->motionCost(nearMotion->state, newState)))
            continue;  // give up on this one and try a new sample

        // V.

        // Create a motion
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, newState);
        motion->parent = nearMotion;  // link q_new to q_near as an edge
        motion->cost = childCost;

        // Add motion to data structure
        nearestNeighbors_->add(motion);

        if (opt_->isCostBetterThan(motion->cost, bestCost_))  // motion->cost is better than the existing best
            bestCost_ = motion->cost;
        if (opt_->isCostBetterThan(worstCost_, motion->cost))  // motion->cost is worse than the existing worst
            worstCost_ = motion->cost;

        // VI.

        // Check if this motion is the goal
        double distToGoal = 0.0;
        bool isSatisfied = goal->isSatisfied(motion->state, &distToGoal);
        if (isSatisfied)
        {
            approxDifference = distToGoal;  // the tolerated error distance btw state and goal
            solution = motion;              // set the final solution
            break;
        }

        // Is this the closest solution we've found so far
        if (distToGoal < approxDifference)
        {
            approxDifference = distToGoal;
            approxSolution = motion;
        }

    }  // end of solver sampling loop

    // Finish solution processing --------------------------------------------------------------------

    bool solved = false;
    bool approximate = false;

    // Substitute an empty solution with the best approximation
    if (solution == nullptr)
    {
        solution = approxSolution;
        approximate = true;
    }

    // Generate solution path for real/approx solution
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

        pdef_->addSolutionPath(path, approximate, approxDifference, getName());
        solved = true;
    }

    // Clean up ---------------------------------------------------------------------------------------

    si_->freeState(interpolatedState);
    if (randMotion->state)
        si_->freeState(randMotion->state);
    delete randMotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nearestNeighbors_->size());

    return {solved, approximate};
}

void ompl::geometric::TRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nearestNeighbors_)
        nearestNeighbors_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

bool ompl::geometric::TRRT::transitionTest(const base::Cost &motionCost)
{
    // Disallow any cost that is not better than the cost threshold
    if (!opt_->isCostBetterThan(motionCost, costThreshold_))
        return false;

    // Always accept if the cost is near or below zero
    if (motionCost.value() < 1e-4)
        return true;

    double dCost = motionCost.value();
    double transitionProbability = exp(-dCost / temp_);
    if (transitionProbability > 0.5)
    {
        double costRange = worstCost_.value() - bestCost_.value();
        if (fabs(costRange) > 1e-4)  // Do not divide by zero
            // Successful transition test.  Decrease the temperature slightly
            temp_ /= exp(dCost / (0.1 * costRange));

        return true;
    }

    // The transition failed.  Increase the temperature (slightly)
    temp_ *= tempChangeFactor_;
    return false;
}

bool ompl::geometric::TRRT::minExpansionControl(double randMotionDistance)
{
    if (randMotionDistance > frontierThreshold_)
    {
        // participates in the tree expansion
        ++frontierCount_;

        return true;
    }
    else
    {
        // participates in the tree refinement

        // check our ratio first before accepting it
        if ((double)nonfrontierCount_ / (double)frontierCount_ > frontierNodeRatio_)
            // reject this node as being too much refinement
            return false;

        ++nonfrontierCount_;
        return true;
    }
}
