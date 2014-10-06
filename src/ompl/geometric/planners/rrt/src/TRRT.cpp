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

/* Author: Dave Coleman */

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

    goalBias_ = 0.05;
    maxDistance_ = 0.0; // set in setup()
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range", this, &TRRT::setRange, &TRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &TRRT::setGoalBias, &TRRT::getGoalBias, "0.:.05:1.");

    // TRRT Specific Variables
    frontierThreshold_ = 0.0; // set in setup()
    kConstant_ = 0.0; // set in setup()
    maxStatesFailed_ = 10; // threshold for when to start increasing the temperatuer
    tempChangeFactor_ = 2.0; // how much to decrease or increase the temp each time
    minTemperature_ = 10e-10; // lower limit of the temperature change
    initTemperature_ = 10e-6; // where the temperature starts out
    frontierNodeRatio_ = 0.1; // 1/10, or 1 nonfrontier for every 10 frontier

    Planner::declareParam<unsigned int>("max_states_failed", this, &TRRT::setMaxStatesFailed, &TRRT::getMaxStatesFailed, "1:1000");
    Planner::declareParam<double>("temp_change_factor", this, &TRRT::setTempChangeFactor, &TRRT::getTempChangeFactor,"0.:.1:10.");
    Planner::declareParam<double>("min_temperature", this, &TRRT::setMinTemperature, &TRRT::getMinTemperature);
    Planner::declareParam<double>("init_temperature", this, &TRRT::setInitTemperature, &TRRT::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold", this, &TRRT::setFrontierThreshold, &TRRT::getFrontierThreshold);
    Planner::declareParam<double>("frontierNodeRatio", this, &TRRT::setFrontierNodeRatio, &TRRT::getFrontierNodeRatio);
    Planner::declareParam<double>("k_constant", this, &TRRT::setKConstant, &TRRT::getKConstant);
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
    lastGoalMotion_ = NULL;

    // Clear TRRT specific variables ---------------------------------------------------------
    numStatesFailed_ = 0;
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1; // init to 1 to prevent division by zero error
}

void ompl::geometric::TRRT::setup()
{
    Planner::setup();
    tools::SelfConfig selfConfig(si_, getName());

    bool usingDefaultObjective = false;
    if (!pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified.", getName().c_str());
        usingDefaultObjective = true;
    }
    else 
    {
        usingDefaultObjective = false;
    }

    if (usingDefaultObjective)
    {
        opt_.reset(new base::MechanicalWorkOptimizationObjective(si_));
        OMPL_INFORM("%s: Defaulting to optimizing path length.", getName().c_str());
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

    // Autoconfigure the K constant
    if (kConstant_ < std::numeric_limits<double>::epsilon())
    {
        // Find the average cost of states by sampling
        double averageCost = opt_->averageStateCost(magic::TEST_STATE_COUNT).value();
        kConstant_ = averageCost;
        OMPL_DEBUG("%s: K constant detected to be %lf", getName().c_str(), kConstant_);
    }

    // Create the nearest neighbor function the first time setup is run
    if (!nearestNeighbors_)
        nearestNeighbors_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));

    // Set the distance function
    nearestNeighbors_->setDistanceFunction(boost::bind(&TRRT::distanceFunction, this, _1, _2));

    // Setup TRRT specific variables ---------------------------------------------------------
    numStatesFailed_ = 0;
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1; // init to 1 to prevent division by zero error
}

void ompl::geometric::TRRT::freeMemory()
{
    // Delete all motions, states and the nearest neighbors data structure
    if (nearestNeighbors_)
    {
        std::vector<Motion*> motions;
        nearestNeighbors_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus
ompl::geometric::TRRT::solve(const base::PlannerTerminationCondition &plannerTerminationCondition)
{
    // Basic error checking
    checkValidity();

    // Goal information
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goalRegion = dynamic_cast<base::GoalSampleableRegion*>(goal);

    // Input States ---------------------------------------------------------------------------------

    // Loop through valid input states and add to tree
    while (const base::State *state = pis_.nextStart())
    {
        // Allocate memory for a new start state motion based on the "space-information"-size
        Motion *motion = new Motion(si_);

        // Copy destination <= source
        si_->copyState(motion->state, state);

        // Set cost for this start state
        motion->cost = opt_->stateCost(motion->state);

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
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nearestNeighbors_->size());


    // Solver variables ------------------------------------------------------------------------------------

    // the final solution
    Motion *solution  = NULL;
    // the approximate solution, returned if no final solution found
    Motion *approxSolution = NULL;
    // track the distance from goal to closest solution yet found
    double  approxDifference = std::numeric_limits<double>::infinity();

    // distance between states - the intial state and the interpolated state (may be the same)
    double randMotionDistance;
    double motionDistance;

    // Create random motion and a pointer (for optimization) to its state
    Motion *randMotion   = new Motion(si_);
    Motion *nearMotion;

    // STATES
    // The random state
    base::State *randState = randMotion->state;
    // The new state that is generated between states *to* and *from*
    base::State *interpolatedState = si_->allocState(); // Allocates "space information"-sized memory for a state
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
            si_->getStateSpace()->interpolate(nearMotion->state, randState,
                                              maxDistance_ / randMotionDistance, interpolatedState);

            // Update the distance between near and new with the interpolated_state
            motionDistance = si_->distance(nearMotion->state, interpolatedState);

            // Use the interpolated state as the new state
            newState = interpolatedState;
        }
        else
        {
            // Random state is close enough
            newState = randState;

            // Copy the distance
            motionDistance = randMotionDistance;
        }

        // IV.
        // this stage integrates collision detections in the presence of obstacles and checks for collisions

        /** bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
            \brief Incrementally check if the path between two motions is valid. Also compute the last state that was
            valid and the time of that state. The time is used to parametrize the motion from s1 to s2, s1 being at t =
            0 and s2 being at t = 1. This function assumes s1 is valid.
            \param s1 start state of the motion to be checked (assumed to be valid)
            \param s2 final state of the motion to be checked
        */
        if (!si_->checkMotion(nearMotion->state, newState))
            continue; // try a new sample


        // Minimum Expansion Control
        // A possible side effect may appear when the tree expansion toward unexplored regions remains slow, and the
        // new nodes contribute only to refine already explored regions.
        if (!minExpansionControl(randMotionDistance))
        {
            continue; // give up on this one and try a new sample
        }

        base::Cost childCost = opt_->stateCost(newState);

        // Only add this motion to the tree if the tranistion test accepts it
        if (!transitionTest(childCost.value(), nearMotion->cost.value(), motionDistance))
        {
            continue; // give up on this one and try a new sample
        }

        // V.

        // Create a motion
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, newState);
        motion->parent = nearMotion; // link q_new to q_near as an edge
        motion->cost = childCost;

        // Add motion to data structure
        nearestNeighbors_->add(motion);

        // VI.

        // Check if this motion is the goal
        double distToGoal = 0.0;
        bool isSatisfied = goal->isSatisfied(motion->state, &distToGoal);
        if (isSatisfied)
        {
            approxDifference = distToGoal; // the tolerated error distance btw state and goal
            solution = motion; // set the final solution
            break;
        }

        // Is this the closest solution we've found so far
        if (distToGoal < approxDifference)
        {
            approxDifference = distToGoal;
            approxSolution = motion;
        }

    } // end of solver sampling loop


    // Finish solution processing --------------------------------------------------------------------

    bool solved = false;
    bool approximate = false;

    // Substitute an empty solution with the best approximation
    if (solution == NULL)
    {
        solution = approxSolution;
        approximate = true;
    }

    // Generate solution path for real/approx solution
    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);

        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxDifference, getName());
        solved = true;
    }

    // Clean up ---------------------------------------------------------------------------------------

    si_->freeState(interpolatedState);
    if (randMotion->state)
        si_->freeState(randMotion->state);
    delete randMotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nearestNeighbors_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::TRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nearestNeighbors_)
        nearestNeighbors_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}

bool ompl::geometric::TRRT::transitionTest(double childCost, double parentCost, double distance)
{
    // Always accept if new state has same or lower cost than old state
    if (childCost <= parentCost)
        return true;

    // Difference in cost
    double costSlope = (childCost - parentCost) / distance;

    // The probability of acceptance of a new configuration is defined by comparing its cost c_j
    // relatively to the cost c_i of its parent in the tree. Based on the Metropolis criterion.
    double transitionProbability = 1.; // if cost_slope is <= 0, probabilty is 1

    // Only return at end
    bool result = false;

    // Calculate tranision probabilty
    if (costSlope > 0)
    {
        transitionProbability = exp(-costSlope / (kConstant_ * temp_));
    }

    // Check if we can accept it
    if (rng_.uniform01() <= transitionProbability)
    {
        if (temp_ > minTemperature_)
        {
            temp_ /= tempChangeFactor_;

            // Prevent temp_ from getting too small
            if (temp_ < minTemperature_)
            {
                temp_ = minTemperature_;
            }
        }

        numStatesFailed_ = 0;

        result = true;
    }
    else
    {
        // State has failed
        if (numStatesFailed_ >= maxStatesFailed_)
        {
            temp_ *= tempChangeFactor_;
            numStatesFailed_ = 0;
        }
        else
        {
            ++numStatesFailed_;
        }

    }

    return result;
}

bool ompl::geometric::TRRT::minExpansionControl(double randMotionDistance)
{
    // Decide to accept or not
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
        {
            // Increment so that the temperature rises faster
            ++numStatesFailed_;

            // reject this node as being too much refinement
            return false;
        }
        else
        {
            ++nonfrontierCount_;
            return true;
        }
    }
}
