/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#include "ompl/geometric/planners/rrt/STRRTstar.h"
#include <ompl/util/Exception.h>

ompl::geometric::STRRTstar::STRRTstar(const ompl::base::SpaceInformationPtr &si)
  : Planner(si, "SpaceTimeRRT"), sampler_(&(*si), startMotion_, goalMotions_, newBatchGoalMotions_, sampleOldBatch_)
{
    if (std::dynamic_pointer_cast<ompl::base::SpaceTimeStateSpace>(si->getStateSpace()) == nullptr) {
        OMPL_ERROR("%s: State Space needs to be of type SpaceTimeStateSpace.", getName().c_str());
        throw ompl::Exception("Non-SpaceTimeStateSpace");
    }
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;
    Planner::declareParam<double>("range", this, &STRRTstar::setRange, &STRRTstar::getRange, "0.:1.:10000.");
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::STRRTstar::~STRRTstar()
{
    freeMemory();
}

void ompl::geometric::STRRTstar::setup()
{
    Planner::setup();
    ompl::tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(new ompl::NearestNeighborsLinear<base::Motion *>());
    if (!tGoal_)
        tGoal_.reset(new ompl::NearestNeighborsLinear<base::Motion *>());
    tStart_->setDistanceFunction([this](const base::Motion *a, const base::Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const base::Motion *a, const base::Motion *b) { return distanceFunction(a, b); });

    if (si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->getTimeComponent()->isBounded())
    {
        upperTimeBound_ =
            si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->getTimeComponent()->getMaxTimeBound();
        isTimeBounded_ = true;
    }
    else
    {
        upperTimeBound_ = std::numeric_limits<double>::infinity();
        isTimeBounded_ = false;
    }
    initialTimeBound_ = upperTimeBound_;

    si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->updateEpsilon();
    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void ompl::geometric::STRRTstar::freeMemory()
{
    std::vector<base::Motion *> motions;

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

    if (tempState_)
        si_->freeState(tempState_);
}

ompl::base::PlannerStatus ompl::geometric::STRRTstar::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const ompl::base::State *st = pis_.nextStart())
    {
        auto *motion = new base::Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
        startMotion_ = motion;
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: base::Motion planning start tree could not be initialized!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_GOAL;
    }

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    std::vector<base::Motion *> nbh;
    const ompl::base::ReportIntermediateSolutionFn intermediateSolutionCallback =
        pdef_->getIntermediateSolutionCallback();

    base::Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new base::Motion(si_);
    ompl::base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    // samples to fill the current batch
    unsigned int batchSize = initialBatchSize_;

    // number of samples in the current batch
    int numBatchSamples = static_cast<int>(tStart_->size() + tGoal_->size());  // number of samples in the current batch

    // number of goal samples in the new batch region
    int newBatchGoalSamples = 0;

    bool firstBatch = true;

    // probability to sample the old batch region
    double oldBatchSampleProb = 1.0;

    // Time Bound factor for the old batch.
    double oldBatchTimeBoundFactor = initialTimeBoundFactor_;

    // Time Bound factor for the new batch.
    double newBatchTimeBoundFactor = initialTimeBoundFactor_;

    bool forceGoalSample = true;

    OMPL_INFORM("%s: Starting planning with time bound factor %.2f", getName().c_str(), newBatchTimeBoundFactor);

    while (!ptc)
    {
        numIterations_++;
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        // batch is full
        if (!isTimeBounded_ && (unsigned int)numBatchSamples >= batchSize)
        {
            if (firstBatch)
            {
                firstBatch = false;
                oldBatchSampleProb = 0.5 * (1 / timeBoundFactorIncrease_);
            }
            increaseTimeBound(false, oldBatchTimeBoundFactor, newBatchTimeBoundFactor, startTree,
                              batchSize, numBatchSamples);
            // transfer new batch goals to old batch
            if (!newBatchGoalMotions_.empty())
            {
                goalMotions_.insert(goalMotions_.end(), newBatchGoalMotions_.begin(), newBatchGoalMotions_.end());
                newBatchGoalMotions_.clear();
            }
            continue;
        }

        // determine whether the old or new batch is sampled
        sampleOldBatch_ =
            (firstBatch || isTimeBounded_ || !sampleUniformForUnboundedTime_ || rng_.uniform01() <= oldBatchSampleProb);

        // *** Begin Goal Sampling ***

        ompl::base::State *goalState{nullptr};
        if (sampleOldBatch_)
        {
            // sample until successful or time is up
            if (goalMotions_.empty() && isTimeBounded_)
                goalState = nextGoal(ptc, oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
            // sample for n tries, with n = batch size
            else if (goalMotions_.empty() && !isTimeBounded_)
            {
                goalState = nextGoal(static_cast<int>(batchSize), oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
                // the goal region is most likely blocked for this time period -> increase upper time bound
                if (goalState == nullptr)
                {
                    increaseTimeBound(true, oldBatchTimeBoundFactor, newBatchTimeBoundFactor, startTree,
                                      batchSize, numBatchSamples);
                    continue;
                }
            }
            // sample for a single try
            else if (forceGoalSample ||
                     goalMotions_.size() < (tGoal_->size() - newBatchGoalSamples) / goalStateSampleRatio_)
            {
                goalState = nextGoal(1, oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
                forceGoalSample = false;
            }
        }
        else
        {
            if (newBatchGoalMotions_.empty())
            {
                goalState = nextGoal(static_cast<int>(batchSize), oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
                // the goal region is most likely blocked for this time period -> increase upper time bound
                if (goalState == nullptr)
                {
                    increaseTimeBound(false, oldBatchTimeBoundFactor, newBatchTimeBoundFactor, startTree,
                                      batchSize, numBatchSamples);
                    continue;
                }
            }
            else if (forceGoalSample || newBatchGoalMotions_.size() < (unsigned long)(newBatchGoalSamples / goalStateSampleRatio_))
            {
                goalState = nextGoal(1, oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
                forceGoalSample = false;
            }
        }

        if (goalState != nullptr)
        {
            auto *motion = new base::Motion(si_);
            si_->copyState(motion->state, goalState);
            motion->root = motion->state;
            tGoal_->add(motion);
            if (sampleOldBatch_)
                goalMotions_.push_back(motion);
            else
            {
                newBatchGoalMotions_.push_back(motion);
                newBatchGoalSamples++;
            }

            minimumTime_ =
                std::min(minimumTime_, si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->timeToCoverDistance(
                                           startMotion_->state, goalState));
            numBatchSamples++;
        }

        // *** End Goal Sampling ***

        /* sample random state */
        bool success = sampler_.sample(rstate);
        if (!success)
        {
            forceGoalSample = true;
            continue;
        }

        // EXTEND
        GrowState gs = growTree(tree, tgi, rmotion, nbh, false);
        if (gs != TRAPPED)
        {
            numBatchSamples++;
            /* remember which motion was just added */
            base::Motion *addedMotion = tgi.xmotion;
            base::Motion *startMotion;
            base::Motion *goalMotion;

            /* rewire the goal tree */
            bool newSolution = false;
            if (!tgi.start && rewireState_ != OFF)
            {
                newSolution = rewireGoalTree(addedMotion);
                if (newSolution)
                {
                    // find connection point
                    std::queue<base::Motion *> queue;
                    queue.push(addedMotion);
                    while (!queue.empty())
                    {
                        if (queue.front()->connectionPoint != nullptr)
                        {
                            goalMotion = queue.front();
                            startMotion = queue.front()->connectionPoint;
                            break;
                        }
                        else
                        {
                            for (base::Motion *c : queue.front()->children)
                                queue.push(c);
                        }
                        queue.pop();
                    }
                }
            }

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            tgi.start = startTree;

            /* attempt to connect trees, if rewiring didn't find a new solution */
            // CONNECT
            if (!newSolution)
            {
                int totalSamples = static_cast<int>(tStart_->size() + tGoal_->size());
                GrowState gsc = growTree(otherTree, tgi, rmotion, nbh, true);
                if (gsc == REACHED)
                {
                    newSolution = true;
                    startMotion = startTree ? tgi.xmotion : addedMotion;
                    goalMotion = startTree ? addedMotion : tgi.xmotion;
                    // it must be the case that either the start tree or the goal tree has made some progress
                    // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                    // on the solution path
                    if (startMotion->parent != nullptr)
                        startMotion = startMotion->parent;
                    else
                        goalMotion = goalMotion->parent;
                }
                numBatchSamples += static_cast<int>(tStart_->size() + tGoal_->size()) - totalSamples;
            }

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (newSolution && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                constructSolution(startMotion, goalMotion, intermediateSolutionCallback, ptc);
                solved = true;
                if (ptc || upperTimeBound_ == minimumTime_)
                    break;  // first solution is enough or optimal solution is found
                // continue to look for solutions with the narrower time bound until the termination condition is met
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (!startTree)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<base::Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
    }
    if (solved)
    {
        // Add the solution path.
        ompl::base::PlannerSolution psol(bestSolution_);
        psol.setPlannerName(getName());

        ompl::base::OptimizationObjectivePtr optimizationObjective = std::make_shared<ompl::base::MinimizeArrivalTime>(si_);
        psol.setOptimized(optimizationObjective, ompl::base::Cost(bestTime_), false);
        pdef_->addSolutionPath(psol);
    }

    return solved ? ompl::base::PlannerStatus::EXACT_SOLUTION : ompl::base::PlannerStatus::TIMEOUT;
}

ompl::geometric::STRRTstar::GrowState ompl::geometric::STRRTstar::growTree(
    ompl::geometric::STRRTstar::TreeData &tree, ompl::geometric::STRRTstar::TreeGrowingInfo &tgi,
    base::Motion *rmotion, std::vector<base::Motion *> &nbh, bool connect)
{
    // If connect, advance from single nearest neighbor until the random state is reached or trapped
    if (connect)
    {
        GrowState gsc = ADVANCED;
        while (gsc == ADVANCED)
        {
            // get nearest motion
            base::Motion *nmotion = tree->nearest(rmotion);
            gsc = growTreeSingle(tree, tgi, rmotion, nmotion);
        }
        return gsc;
    }
    if (rewireState_ == OFF)
    {
        base::Motion *nmotion = tree->nearest(rmotion);
        return growTreeSingle(tree, tgi, rmotion, nmotion);
    }
    // get Neighborhood of random state
    getNeighbors(tree, rmotion, nbh);
    // in start tree sort by distance
    if (tgi.start)
    {
        std::sort(nbh.begin(), nbh.end(),
                  [this, &rmotion](base::Motion *a, base::Motion *b)
                  { return si_->distance(a->state, rmotion->state) < si_->distance(b->state, rmotion->state); });
    }
    // in goal tree sort by time of root node
    else
    {
        std::sort(
            nbh.begin(), nbh.end(),
            [](base::Motion *a, base::Motion *b)
            {
                auto t1 =
                    a->root->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
                auto t2 =
                    b->root->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
                return t1 < t2;
            });
    }

    // attempt to grow the tree for all neighbors in sorted order
    GrowState gs = TRAPPED;
    auto rt = rmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
    for (base::Motion *nmotion : nbh)
    {
        auto nt =
            nmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
        // trees grow only in one direction in time
        if ((tgi.start && nt > rt) || (!tgi.start && nt < rt))
            continue;
        gs = growTreeSingle(tree, tgi, rmotion, nmotion);
        if (gs != TRAPPED)
            return gs;
    }
    // when radius_ is used for neighborhood calculation, the neighborhood might be empty
    if (nbh.empty())
    {
        base::Motion *nmotion = tree->nearest(rmotion);
        return growTreeSingle(tree, tgi, rmotion, nmotion);
    }
    // can't grow Tree
    return gs;
}

ompl::geometric::STRRTstar::GrowState ompl::geometric::STRRTstar::growTreeSingle(
    ompl::geometric::STRRTstar::TreeData &tree, ompl::geometric::STRRTstar::TreeGrowingInfo &tgi,
    base::Motion *rmotion, base::Motion *nmotion)
{
    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    ompl::base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);

    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;
        dstate = tgi.xstate;
        reach = false;
    }

    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                                   si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (!validMotion)
        return TRAPPED;

    auto *motion = new base::Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nmotion;
    motion->root = nmotion->root;
    motion->parent->children.push_back(motion);
    tree->add(motion);
    tgi.xmotion = motion;

    return reach ? REACHED : ADVANCED;
}

void ompl::geometric::STRRTstar::increaseTimeBound(bool hasSameBounds, double &oldBatchTimeBoundFactor,
                                                   double &newBatchTimeBoundFactor, bool &startTree,
                                                   unsigned int &batchSize, int &numBatchSamples)
{
    oldBatchTimeBoundFactor = hasSameBounds ? newBatchTimeBoundFactor * timeBoundFactorIncrease_ : newBatchTimeBoundFactor;
    newBatchTimeBoundFactor *= timeBoundFactorIncrease_;
    startTree = true;
    if (sampleUniformForUnboundedTime_)
        batchSize = std::ceil(2.0 * (timeBoundFactorIncrease_ - 1.0) *
                              static_cast<double>(tStart_->size() + tGoal_->size()));
    numBatchSamples = 0;
    OMPL_INFORM("%s: Increased time bound factor to %.2f", getName().c_str(), newBatchTimeBoundFactor);
}

void ompl::geometric::STRRTstar::constructSolution(
    base::Motion *startMotion, base::Motion *goalMotion,
    const ompl::base::ReportIntermediateSolutionFn &intermediateSolutionCallback, const ompl::base::PlannerTerminationCondition& ptc)
{
    if (goalMotion->connectionPoint == nullptr)
    {
        goalMotion->connectionPoint = startMotion;
        base::Motion *tMotion = goalMotion;
        while (tMotion != nullptr)
        {
            tMotion->numConnections++;
            tMotion = tMotion->parent;
        }
    }
    // check whether the found solution is an improvement
    auto newTime =
        goalMotion->root->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
    if (newTime >= upperTimeBound_)
        return;

    numSolutions_++;
    isTimeBounded_ = true;
    if (!newBatchGoalMotions_.empty())
    {
        goalMotions_.insert(goalMotions_.end(), newBatchGoalMotions_.begin(), newBatchGoalMotions_.end());
        newBatchGoalMotions_.clear();
    }

    /* construct the solution path */
    base::Motion *solution = startMotion;
    std::vector<base::Motion *> mpath1;
    while (solution != nullptr)
    {
        mpath1.push_back(solution);
        solution = solution->parent;
    }

    solution = goalMotion;
    std::vector<base::Motion *> mpath2;
    while (solution != nullptr)
    {
        mpath2.push_back(solution);
        solution = solution->parent;
    }

    std::vector<const ompl::base::State *> constPath;

    auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
    path->getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i)
    {
        constPath.push_back(mpath1[i]->state);
        path->append(mpath1[i]->state);
    }
    for (auto &i : mpath2)
    {
        constPath.push_back(i->state);
        path->append(i->state);
    }

    bestSolution_ = path;
    auto reachedGaol = path->getState(path->getStateCount() - 1);
    bestTime_ = reachedGaol->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;

    if (intermediateSolutionCallback)
    {
        intermediateSolutionCallback(this, constPath, ompl::base::Cost(bestTime_));
    }

    // Update Time Limit
    upperTimeBound_ = (bestTime_ - minimumTime_) * optimumApproxFactor_ + minimumTime_;

    if (ptc)
        return;
    // Prune Start and Goal Trees
    pruneStartTree();
    base::Motion *newSolution = pruneGoalTree();

    // loop as long as a new solution is found by rewiring the goal tree
    if (newSolution != nullptr)
        constructSolution(newSolution->connectionPoint, goalMotion, intermediateSolutionCallback, ptc);
}

void ompl::geometric::STRRTstar::pruneStartTree()
{
    std::queue<base::Motion *> queue;

    tStart_->clear();
    tStart_->add(startMotion_);
    for (auto &c : startMotion_->children)
        queue.push(c);
    while (!queue.empty())
    {
        double t = queue.front()
                       ->state->as<ompl::base::CompoundState>()
                       ->as<ompl::base::TimeStateSpace::StateType>(1)
                       ->position;
        double timeToNearestGoal = std::numeric_limits<double>::infinity();
        for (const auto &g : goalMotions_)
        {
            double deltaT = si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->timeToCoverDistance(
                queue.front()->state, g->state);
            if (deltaT < timeToNearestGoal)
                timeToNearestGoal = deltaT;
        }
        // base::Motion is still valid, re-add to tree
        if (t + timeToNearestGoal <= upperTimeBound_)
        {
            tStart_->add(queue.front());
            for (auto &c : queue.front()->children)
                queue.push(c);
        }
        // base::Motion is invalid due to the new time limit, delete motion
        else
        {
            // Remove the motion from its parent
            removeFromParent(queue.front());

            // for deletion first construct list of all descendants
            std::queue<base::Motion *> deletionQueue;
            std::vector<base::Motion *> deletionList;

            deletionQueue.push(queue.front());
            while (!deletionQueue.empty())
            {
                for (auto &c : deletionQueue.front()->children)
                    deletionQueue.push(c);
                deletionList.push_back(deletionQueue.front());
                deletionQueue.pop();
            }

            // then free all descendants
            for (auto &m : deletionList)
            {
                // Erase the actual motion
                // First free the state
                if (m->state)
                    si_->freeState(m->state);
                // then delete the pointer
                delete m;
            }
        }
        // finally remove motion from the queue
        queue.pop();
    }
}

ompl::base::Motion *ompl::geometric::STRRTstar::pruneGoalTree()
{
    // it's possible to get multiple new solutions during the rewiring process. Store the best.
    double bestSolutionTime = upperTimeBound_;
    base::Motion *solutionMotion{nullptr};

    tGoal_->clear();
    std::vector<base::Motion *> validGoals;
    std::vector<base::Motion *> invalidGoals;

    // re-add goals with smallest time first
    std::sort(
        goalMotions_.begin(), goalMotions_.end(),
        [](base::Motion *a, base::Motion *b)
        {
            return a->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position <
                   b->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
        });
    for (auto &m : goalMotions_)
    {
        double t = m->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
        // add goal with all descendants to the tree
        if (t <= upperTimeBound_)
        {
            tGoal_->add(m);
            addDescendants(m, tGoal_);
            validGoals.push_back(m);
        }
        // try to rewire descendants to a valid goal
        else
        {
            invalidGoals.push_back(m);
            std::queue<base::Motion *> queue;
            for (auto &c : m->children)
                queue.push(c);
            while (!queue.empty())
            {
                bool addedToTree = false;
                if (tGoal_->size() != 0)
                {
                    double costToGo = std::numeric_limits<double>::infinity();
                    double costSoFar = queue.front()
                                           ->state->as<ompl::base::CompoundState>()
                                           ->as<ompl::base::TimeStateSpace::StateType>(1)
                                           ->position;
                    for (auto &g : validGoals)
                    {
                        auto deltaT = si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->timeToCoverDistance(
                            queue.front()->state, g->state);
                        if (deltaT < costToGo)
                            costToGo = deltaT;
                    }
                    // try to rewire to the nearest neighbor

                    if (costSoFar + costToGo <= upperTimeBound_)
                    {
                        TreeGrowingInfo tgi{};
                        tgi.xstate = si_->allocState();
                        tgi.start = false;
                        std::vector<base::Motion *> nbh;
                        GrowState gsc = growTree(tGoal_, tgi, queue.front(), nbh, true);
                        // connection successful, add all descendants and check if a new solution was found.
                        if (gsc == REACHED)
                        {
                            // the motion was copied and added to the tree with a new parent
                            // adjust children and parent pointers
                            tgi.xmotion->children = queue.front()->children;
                            for (auto &c : tgi.xmotion->children)
                            {
                                c->parent = tgi.xmotion;
                            }
                            tgi.xmotion->connectionPoint = queue.front()->connectionPoint;
                            tgi.xmotion->numConnections = queue.front()->numConnections;
                            base::Motion *p = tgi.xmotion->parent;
                            while (p != nullptr)
                            {
                                p->numConnections += tgi.xmotion->numConnections;
                                p = p->parent;
                            }
                            addDescendants(tgi.xmotion, tGoal_);
                            // new solution found
                            if (tgi.xmotion->numConnections > 0 &&
                                tgi.xmotion->root->as<ompl::base::CompoundState>()
                                        ->as<ompl::base::TimeStateSpace::StateType>(1)
                                        ->position < bestSolutionTime)
                            {
                                bestSolutionTime = tgi.xmotion->root->as<ompl::base::CompoundState>()
                                                       ->as<ompl::base::TimeStateSpace::StateType>(1)
                                                       ->position;
                                solutionMotion = computeSolutionMotion(tgi.xmotion);
                            }
                            addedToTree = true;
                        }
                    }
                }
                // Free motion and state
                if (!addedToTree)
                {
                    // add children to queue, so they might be rewired
                    for (auto &c : queue.front()->children)
                        queue.push(c);
                }
                // Erase the actual motion
                // First free the state
                if (queue.front()->state)
                    si_->freeState(queue.front()->state);
                // then delete the pointer
                delete queue.front();

                queue.pop();
            }
        }
    }

    removeInvalidGoals(invalidGoals);
    return solutionMotion;
}

ompl::base::Motion*
ompl::geometric::STRRTstar::computeSolutionMotion(base::Motion *motion)
{
    std::queue<base::Motion *> connectionQueue;
    connectionQueue.push(motion);
    while (!connectionQueue.empty())
    {
        if (connectionQueue.front()->connectionPoint != nullptr)
        {
            return connectionQueue.front();
        }
        else
        {
            for (base::Motion *c : connectionQueue.front()->children)
                connectionQueue.push(c);
        }
        connectionQueue.pop();
    }
    // suppress compiler warning
    return nullptr;
}

void ompl::geometric::STRRTstar::removeInvalidGoals(const std::vector<base::Motion *> &invalidGoals)
{
    for (auto &g : invalidGoals)
    {
        for (auto it = goalMotions_.begin(); it != goalMotions_.end(); ++it)
        {
            if (*it == g)
            {
                goalMotions_.erase(it);
                break;
            }
        }
        if (g->state)
            si_->freeState(g->state);
        delete g;
    }
}


void ompl::geometric::STRRTstar::clear()
{
    setup_ = false;
    Planner::clear();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    bestSolution_ = nullptr;
    bestTime_ = std::numeric_limits<double>::infinity();
    minimumTime_ = std::numeric_limits<double>::infinity();
    numIterations_ = 0;
    numSolutions_ = 0;
    startMotion_ = nullptr;
    goalMotions_.clear();
    newBatchGoalMotions_.clear();
    tempState_ = nullptr;
    sampleOldBatch_ = true;
    upperTimeBound_ = initialTimeBound_;
    isTimeBounded_ = initialTimeBound_ != std::numeric_limits<double>::infinity();
}

void ompl::geometric::STRRTstar::getPlannerData(ompl::base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<base::Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(ompl::base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state, 1),
                         ompl::base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(ompl::base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(ompl::base::PlannerDataVertex(motion->state, 2),
                         ompl::base::PlannerDataVertex(motion->parent->state, 2));
        }
        // add edges connecting the two trees
        if (motion->connectionPoint != nullptr)
            data.addEdge(data.vertexIndex(motion->connectionPoint->state), data.vertexIndex(motion->state));
    }

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

void ompl::geometric::STRRTstar::removeFromParent(base::Motion *m)
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

/**
 * Adds all descendants of a motion to a given tree.
 *
 * @param m The motion, which descendants are added
 * @param tree The tree that the motions are added to

 */
void ompl::geometric::STRRTstar::addDescendants(base::Motion *m,
                                                const ompl::geometric::STRRTstar::TreeData &tree)
{
    std::queue<base::Motion *> queue;
    for (auto &c : m->children)
        queue.push(c);
    while (!queue.empty())
    {
        for (auto &c : queue.front()->children)
            queue.push(c);
        queue.front()->root = m->root;
        tree->add(queue.front());
        queue.pop();
    }
}

void ompl::geometric::STRRTstar::getNeighbors(ompl::geometric::STRRTstar::TreeData &tree,
                                              base::Motion *motion,
                                              std::vector<base::Motion *> &nbh) const
{
    auto card = static_cast<double>(tree->size() + 1u);
    if (rewireState_ == RADIUS)
    {
        // r = min( r_rrt * (log(card(V))/card(V))^(1 / d + 1), distance)
        // for the formula change of the RRTStar paper, see 'Revisiting the asymptotic optimality of RRT*'
        double r = std::min(maxDistance_, r_rrt_ * std::pow(log(card) / card,
                                                            1.0 / 1.0 + static_cast<double>(si_->getStateDimension())));
        tree->nearestR(motion, r, nbh);
    }
    else if (rewireState_ == KNEAREST)
    {
        // k = k_rrt * log(card(V))
        unsigned int k = std::ceil(k_rrt_ * log(card));
        tree->nearestK(motion, k, nbh);
    }
}

bool ompl::geometric::STRRTstar::rewireGoalTree(base::Motion *addedMotion)
{
    bool solved = false;
    std::vector<base::Motion *> nbh;
    getNeighbors(tGoal_, addedMotion, nbh);
    double nodeT =
        addedMotion->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
    double goalT =
        addedMotion->root->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;

    for (base::Motion *otherMotion : nbh)
    {
        double otherNodeT =
            otherMotion->state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
        double otherGoalT =
            otherMotion->root->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
        // rewire, if goal time is improved and the otherMotion node can be connected to the added node
        if (otherNodeT < nodeT && goalT < otherGoalT && si_->checkMotion(otherMotion->state, addedMotion->state))
        {
            if (otherMotion->numConnections > 0)
            {
                base::Motion *p = otherMotion->parent;
                while (p != nullptr)
                {
                    p->numConnections--;
                    p = p->parent;
                }
            }
            removeFromParent(otherMotion);
            otherMotion->parent = addedMotion;
            otherMotion->root = addedMotion->root;
            addedMotion->children.push_back(otherMotion);
            // increase connection count of new ancestors
            if (otherMotion->numConnections > 0)
            {
                base::Motion *p = otherMotion->parent;
                while (p != nullptr)
                {
                    p->numConnections++;
                    p = p->parent;
                }
                if (otherMotion->root->as<ompl::base::CompoundState>()
                        ->as<ompl::base::TimeStateSpace::StateType>(1)
                        ->position < upperTimeBound_)
                {
                    solved = true;
                }
            }
        }
    }

    return solved;
}

void ompl::geometric::STRRTstar::calculateRewiringLowerBounds()
{
    const auto dim = static_cast<double>(si_->getStateDimension());

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // prunedMeasure_ is set to si_->getSpaceMeasure();
    r_rrt_ = rewireFactor_ * std::pow(2 * (1.0 + 1.0 / dim) *
                                          (si_->getSpaceMeasure() / ompl::unitNBallMeasure(si_->getStateDimension())),
                                      1.0 / dim);

    // k_rrg > e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * boost::math::constants::e<double>() * (1.0 + 1.0 / dim);
}

bool ompl::geometric::STRRTstar::sampleGoalTime(ompl::base::State *goal, double oldBatchTimeBoundFactor,
                                                double newBatchTimeBoundFactor)
{
    double ltb, utb;
    double minTime =
        si_->getStateSpace()->as<ompl::base::SpaceTimeStateSpace>()->timeToCoverDistance(startMotion_->state, goal);
    if (isTimeBounded_)
    {
        ltb = minTime;
        utb = upperTimeBound_;
    }
    else if (sampleOldBatch_)
    {
        ltb = minTime;
        utb = minTime * oldBatchTimeBoundFactor;
    }
    else
    {
        ltb = minTime * oldBatchTimeBoundFactor;
        utb = minTime * newBatchTimeBoundFactor;
    }

    if (ltb > utb)
        return false;  // goal can't be reached in time

    double time = ltb == utb ? ltb : rng_.uniformReal(ltb, utb);
    goal->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position = time;
    return true;
}

ompl::base::State *ompl::geometric::STRRTstar::nextGoal(int n, double oldBatchTimeBoundFactor,
                                                        double newBatchTimeBoundFactor)
{
    static ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerNonTerminatingCondition();
    return nextGoal(ptc, n, oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
}

ompl::base::State *ompl::geometric::STRRTstar::nextGoal(const ompl::base::PlannerTerminationCondition &ptc,
                                                        double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor)
{
    return nextGoal(ptc, -1, oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
}

ompl::base::State *ompl::geometric::STRRTstar::nextGoal(const ompl::base::PlannerTerminationCondition &ptc, int n,
                                                        double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor)
{
    if (pdef_->getGoal() != nullptr)
    {
        const ompl::base::GoalSampleableRegion *goal = pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION) ?
                                                           pdef_->getGoal()->as<ompl::base::GoalSampleableRegion>() :
                                                           nullptr;

        if (goal != nullptr)
        {
            if (tempState_ == nullptr)
                tempState_ = si_->allocState();
            int tryCount = 0;
            do
            {
                goal->sampleGoal(tempState_);  // sample space component
                // sample time component dependant on sampled space
                bool inTime = sampleGoalTime(tempState_, oldBatchTimeBoundFactor, newBatchTimeBoundFactor);
                bool bounds = inTime && si_->satisfiesBounds(tempState_);
                bool valid = bounds && si_->isValid(tempState_);
                if (valid)
                {
                    return tempState_;
                }
            } while (!ptc.eval() && ++tryCount != n);
        }
    }

    return nullptr;
}

void ompl::geometric::STRRTstar::setRange(double distance)
{
    maxDistance_ = distance;
}

double ompl::geometric::STRRTstar::getRange() const
{
    return maxDistance_;
}

double ompl::geometric::STRRTstar::getOptimumApproxFactor() const
{
    return optimumApproxFactor_;
}

void ompl::geometric::STRRTstar::setOptimumApproxFactor(double optimumApproxFactor)
{
    if (optimumApproxFactor <= 0 || optimumApproxFactor > 1)
    {
        OMPL_ERROR("%s: The optimum approximation factor needs to be between 0 and 1.", getName().c_str());
    }
    optimumApproxFactor_ = optimumApproxFactor;
}

std::string ompl::geometric::STRRTstar::getRewiringState() const
{
    std::vector<std::string> s{"Radius", "KNearest", "Off"};
    return s[rewireState_];
}

void ompl::geometric::STRRTstar::setRewiringToOff()
{
    rewireState_ = OFF;
}

void ompl::geometric::STRRTstar::setRewiringToRadius()
{
    rewireState_ = RADIUS;
}

void ompl::geometric::STRRTstar::setRewiringToKNearest()
{
    rewireState_ = KNEAREST;
}

double ompl::geometric::STRRTstar::getRewireFactor() const
{
    return rewireFactor_;
}

void ompl::geometric::STRRTstar::setRewireFactor(double v)
{
    if (v <= 1)
    {
        OMPL_ERROR("%s: Rewire Factor needs to be greater than 1.", getName().c_str());
    }
    rewireFactor_ = v;
}

unsigned int ompl::geometric::STRRTstar::getBatchSize() const
{
    return initialBatchSize_;
}

void ompl::geometric::STRRTstar::setBatchSize(int v)
{
    if (v < 1)
    {
        OMPL_ERROR("%s: Batch Size needs to be at least 1.", getName().c_str());
    }
    initialBatchSize_ = v;
}

void ompl::geometric::STRRTstar::setTimeBoundFactorIncrease(double f)
{
    if (f <= 1.0)
    {
        OMPL_ERROR("%s: Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
    }
    timeBoundFactorIncrease_ = f;
}

void ompl::geometric::STRRTstar::setInitialTimeBoundFactor(double f)
{
    if (f <= 1.0)
    {
        OMPL_ERROR("%s: Initial Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
    }
    initialTimeBoundFactor_ = f;
}

void ompl::geometric::STRRTstar::setSampleUniformForUnboundedTime(bool uniform)
{
    sampleUniformForUnboundedTime_ = uniform;
}
