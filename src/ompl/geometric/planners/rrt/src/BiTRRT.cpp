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
*   * Neither the name of the Rice University nor the names of its
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

#include <limits>

#include "ompl/geometric/planners/rrt/BiTRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::BiTRRT::BiTRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "BiTRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &BiTRRT::setRange, &BiTRRT::getRange, "0.:1.:10000.");

    // BiTRRT Specific Variables
    setTempChangeFactor(0.1);  // how much to increase the temp each time

    Planner::declareParam<double>("temp_change_factor", this, &BiTRRT::setTempChangeFactor,
                                  &BiTRRT::getTempChangeFactor, "0.:.1:1.");
    Planner::declareParam<double>("init_temperature", this, &BiTRRT::setInitTemperature, &BiTRRT::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold", this, &BiTRRT::setFrontierThreshold,
                                  &BiTRRT::getFrontierThreshold);
    Planner::declareParam<double>("frontier_node_ratio", this, &BiTRRT::setFrontierNodeRatio,
                                  &BiTRRT::getFrontierNodeRatio);
    Planner::declareParam<double>("cost_threshold", this, &BiTRRT::setCostThreshold, &BiTRRT::getCostThreshold);
}

ompl::geometric::BiTRRT::~BiTRRT()
{
    freeMemory();
}

void ompl::geometric::BiTRRT::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::BiTRRT::clear()
{
    Planner::clear();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<Motion *, Motion *>(nullptr, nullptr);

    // TRRT specific variables
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1;  // init to 1 to prevent division by zero error
    if (opt_)
        bestCost_ = worstCost_ = opt_->identityCost();
}

void ompl::geometric::BiTRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    // Configuring the range of the planner
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
        sc.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    // Configuring nearest neighbors structures for the planning trees
    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                {
                                    return distanceFunction(a, b);
                                });

    // Setup the optimization objective, if it isn't specified
    if (!pdef_ || !pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified.  Defaulting to mechanical work minimization.",
                    getName().c_str());
        opt_ = std::make_shared<base::MechanicalWorkOptimizationObjective>(si_);
    }
    else
        opt_ = pdef_->getOptimizationObjective();

    // Set the threshold that decides if a new node is a frontier node or non-frontier node
    if (frontierThreshold_ < std::numeric_limits<double>::epsilon())
    {
        frontierThreshold_ = si_->getMaximumExtent() * 0.01;
        OMPL_DEBUG("%s: Frontier threshold detected to be %lf", getName().c_str(), frontierThreshold_);
    }

    // initialize TRRT specific variables
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1;  // init to 1 to prevent division by zero error
    bestCost_ = worstCost_ = opt_->identityCost();
    connectionRange_ = 10.0 * si_->getStateSpace()->getLongestValidSegmentLength();
}

ompl::geometric::BiTRRT::Motion *ompl::geometric::BiTRRT::addMotion(const base::State *state, TreeData &tree,
                                                                    Motion *parent)
{
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->cost = opt_->stateCost(motion->state);
    motion->parent = parent;
    motion->root = parent != nullptr ? parent->root : nullptr;

    if (opt_->isCostBetterThan(motion->cost, bestCost_))  // motion->cost is better than the existing best
        bestCost_ = motion->cost;
    if (opt_->isCostBetterThan(worstCost_, motion->cost))  // motion->cost is worse than the existing worst
        worstCost_ = motion->cost;

    // Add start motion to the tree
    tree->add(motion);
    return motion;
}

bool ompl::geometric::BiTRRT::transitionTest(const base::Cost &motionCost)
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

bool ompl::geometric::BiTRRT::minExpansionControl(double dist)
{
    if (dist > frontierThreshold_)  // Exploration
    {
        ++frontierCount_;
        return true;
    }
    // Refinement
    // Check the current ratio first before accepting it
    if ((double)nonfrontierCount_ / (double)frontierCount_ > frontierNodeRatio_)
        return false;

    ++nonfrontierCount_;
    return true;
}

ompl::geometric::BiTRRT::GrowResult ompl::geometric::BiTRRT::extendTree(Motion *nearest, TreeData &tree,
                                                                        Motion *toMotion, Motion *&result)
{
    bool reach = true;

    // Compute the state to extend toward
    bool treeIsStart = (tree == tStart_);
    double d = (treeIsStart ? si_->distance(nearest->state, toMotion->state)
                            : si_->distance(toMotion->state, nearest->state));
    // Truncate the random state to be no more than maxDistance_ from nearest neighbor
    if (d > maxDistance_)
    {
        if (tree == tStart_)
            si_->getStateSpace()->interpolate(nearest->state, toMotion->state, maxDistance_ / d, toMotion->state);
        else
            si_->getStateSpace()->interpolate(toMotion->state, nearest->state, 1.0 - maxDistance_ / d, toMotion->state);
        d = maxDistance_;
        reach = false;
    }

    // Validating the motion
    // If we are in the goal tree, we validate the motion in reverse
    // si_->checkMotion assumes that the first argument is valid, so we must check this explicitly
    // If the motion is valid, check the probabilistic transition test and the
    // expansion control to ensure high quality nodes are added.
    bool validMotion =
        (tree == tStart_ ? si_->checkMotion(nearest->state, toMotion->state) :
                           si_->isValid(toMotion->state) && si_->checkMotion(toMotion->state, nearest->state)) &&
                               transitionTest(tree == tStart_ ? opt_->motionCost(nearest->state, toMotion->state)
                                                              : opt_->motionCost(toMotion->state, nearest->state)) &&
                               minExpansionControl(d);

    if (validMotion)
    {
        result = addMotion(toMotion->state, tree, nearest);
        return reach ? SUCCESS : ADVANCED;
    }

    return FAILED;
}

ompl::geometric::BiTRRT::GrowResult ompl::geometric::BiTRRT::extendTree(Motion *toMotion, TreeData &tree,
                                                                        Motion *&result)
{
    // Nearest neighbor
    Motion *nearest = tree->nearest(toMotion);
    return extendTree(nearest, tree, toMotion, result);
}

bool ompl::geometric::BiTRRT::connectTrees(Motion *nmotion, TreeData &tree, Motion *xmotion)
{
    // Get the nearest state to nmotion in tree (nmotion is NOT in tree)
    Motion *nearest = tree->nearest(nmotion);
    bool treeIsStart = tree == tStart_;
    double dist = (treeIsStart ? si_->distance(nearest->state, nmotion->state)
                               : si_->distance(nmotion->state, nearest->state));

    // Do not attempt a connection if the trees are far apart
    if (dist > connectionRange_)
        return false;

    // Copy the resulting state into our scratch space
    si_->copyState(xmotion->state, nmotion->state);

    // Do not try to connect states directly.  Must chop up the
    // extension into segments, just in case one piece fails
    // the transition test
    GrowResult result;
    Motion *next = nullptr;
    do
    {
        // Extend tree from nearest toward xmotion
        // Store the result into next
        // This function MAY trash xmotion
        result = extendTree(nearest, tree, xmotion, next);

        if (result == ADVANCED)
        {
            nearest = next;

            // xmotion may get trashed during extension, so we reload it here
            si_->copyState(xmotion->state,
                           nmotion->state);  // xmotion may get trashed during extension, so we reload it here
        }
    } while (result == ADVANCED);

    // Successful connection
    if (result == SUCCESS)
    {
        Motion *startMotion = treeIsStart ? next : nmotion;
        Motion *goalMotion = treeIsStart ? nmotion : next;

        // Make sure start-goal pair is valid
        if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {
            // Since we have connected, nmotion->state and next->state have the same value
            // We need to check one of their parents to avoid a duplicate state in the solution path
            // One of these must be true, since we do not ever attempt to connect start and goal directly.
            if (startMotion->parent != nullptr)
                startMotion = startMotion->parent;
            else
                goalMotion = goalMotion->parent;

            connectionPoint_ = std::make_pair(startMotion, goalMotion);
            return true;
        }
    }

    return false;
}

ompl::base::PlannerStatus ompl::geometric::BiTRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // Basic error checking
    checkValidity();

    // Goal information
    base::Goal *goal = pdef_->getGoal().get();
    auto *gsr = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (gsr == nullptr)
    {
        OMPL_ERROR("%s: Goal object does not derive from GoalSampleableRegion", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Loop through the (valid) input states and add them to the start tree
    while (const base::State *state = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, state);
        motion->cost = opt_->stateCost(motion->state);
        motion->root = motion->state;  // this state is the root of a tree

        if (tStart_->size() == 0)  // do not overwrite best/worst from a prior call to solve
            worstCost_ = bestCost_ = motion->cost;

        // Add start motion to the tree
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Start tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Do the same for the goal tree, if it is empty, but only once
    if (tGoal_->size() == 0)
    {
        const base::State *state = pis_.nextGoal(ptc);
        if (state != nullptr)
        {
            Motion *motion = addMotion(state, tGoal_);
            motion->root = motion->state;  // this state is the root of a tree
        }
    }

    if (tGoal_->size() == 0)
    {
        OMPL_ERROR("%s: Goal tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    OMPL_INFORM("%s: Planning started with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    base::StateSamplerPtr sampler = si_->allocStateSampler();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    auto *xmotion = new Motion(si_);
    base::State *xstate = xmotion->state;

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;

    bool solved = false;
    // Planning loop
    while (!ptc)
    {
        // Check if there are more goal states
        if (pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            if (const base::State *state = pis_.nextGoal())
            {
                Motion *motion = addMotion(state, tGoal_);
                motion->root = motion->state;  // this state is the root of a tree
            }
        }

        // Sample a state uniformly at random
        sampler->sampleUniform(rstate);

        Motion *result;                                   // the motion that gets added in extendTree
        if (extendTree(rmotion, tree, result) != FAILED)  // we added something new to the tree
        {
            // Try to connect the other tree to the node we just added
            if (connectTrees(result, otherTree, xmotion))
            {
                // The trees have been connected.  Construct the solution path
                Motion *solution = connectionPoint_.first;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = connectionPoint_.second;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
        }

        std::swap(tree, otherTree);
    }

    si_->freeState(rstate);
    si_->freeState(xstate);
    delete rmotion;
    delete xmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::BiTRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);
    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);
    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    if ((connectionPoint_.first != nullptr) && (connectionPoint_.second != nullptr))
        data.addEdge(data.vertexIndex(connectionPoint_.first->state), data.vertexIndex(connectionPoint_.second->state));
}