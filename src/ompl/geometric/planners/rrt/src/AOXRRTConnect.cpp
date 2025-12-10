/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Queen's University
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
 *   * Neither the name of the Queen's University nor the names of its
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

/* Author: Tyler Wilson */

#include "ompl/geometric/planners/rrt/AOXRRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"

#include <stdexcept>

ompl::geometric::AOXRRTConnect::AOXRRTConnect(const base::SpaceInformationPtr &si)
  : base::Planner(si, "AOXRRTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &AOXRRTConnect::setRange, &AOXRRTConnect::getRange, "0.:1.:10000.");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

ompl::geometric::AOXRRTConnect::~AOXRRTConnect()
{
    freeMemory();
}

void ompl::geometric::AOXRRTConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            /* Store the new objective in the problem def'n */
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));

    reset(true);
}

void ompl::geometric::AOXRRTConnect::freeMemory()
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

void ompl::geometric::AOXRRTConnect::reset(bool solvedProblem)
{
    tStart_->clear();
    tGoal_->clear();

    /* If we have a reasonable cost bound, then use the AOX distance function */
    if (bestCost_.value() < std::numeric_limits<double>::infinity())
    {
        tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return aoxDistanceFunction(a, b); });
        tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return aoxDistanceFunction(a, b); });
    } 
    else /* Use standard distance function if no cost bound is set */
    {
        tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return euclideanDistanceFunction(a, b); });
        tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return euclideanDistanceFunction(a, b); });
    }

    /* Start from the start tree */
    startTree_ = true;
    sampleAttempts = 0;

    /* If we didn't solve the problem, loosen our internal reset conditions
       This is for probabilistic completeness, so that the planner can still
       eventually find a solution that may require more vertices/samples */
    if (!solvedProblem)
    {
        maxInternalSamples += maxInternalSamplesIncrement;
        maxInternalVertices += maxInternalVerticesIncrement;
    } else {
        /* If we solved the problem, reset our internal reset conditions as well */
        maxInternalSamples = maxInternalSamplesIncrement;
        maxInternalVertices = maxInternalVerticesIncrement;
    }
}

void ompl::geometric::AOXRRTConnect::setPathCost(double pc)
{
    bestCost_ = base::Cost(pc);
    opt_->setCostThreshold(bestCost_);
}

ompl::geometric::AOXRRTConnect::Motion *ompl::geometric::AOXRRTConnect::findNeighbour(Motion *sampled_motion,
                                                                                      float rootDist, TreeData &tree)
{
    Motion *nearest_motion;
    std::vector<Motion *> nearest_vec;

    rootDist += rootDistPadding;

    tree->nearestR(sampled_motion, rootDist, nearest_vec);
    if (nearest_vec.empty())
    {
        OMPL_ERROR("%s: Failed to find any neighbours for a state", getName().c_str());
        return nullptr;
    }

    int idx = 0;
    nearest_motion = nearest_vec[idx];
    auto nearest_distance = si_->distance(sampled_motion->state, nearest_motion->state);

    while (nearest_motion->cost > 0 && sampled_motion->cost < nearest_motion->cost + nearest_distance)
    {
        idx++;
        nearest_motion = nearest_vec[idx];
        nearest_distance = si_->distance(sampled_motion->state, nearest_motion->state);
    }

    return nearest_motion;
}

ompl::geometric::AOXRRTConnect::GrowState ompl::geometric::AOXRRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                                   Motion *rmotion)
{
    auto g_hat = tgi.start ? si_->distance(rmotion->state, startState) : si_->distance(rmotion->state, goalState);

    auto *root_motion = new Motion(si_);
    si_->copyState(root_motion->state, tgi.start ? startState : goalState);
    root_motion->cost = 0;

    auto rootDist = tree->getDistanceFunction()(rmotion, root_motion);
    Motion *nmotion = findNeighbour(rmotion, rootDist, tree);

    /* If there were no neighbours */
    if (nmotion == nullptr)
    {
        si_->freeState(root_motion->state);
        delete root_motion;
        return TRAPPED;
    }

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
        {
            si_->freeState(root_motion->state);
            delete root_motion;
            return TRAPPED;
        }

        dstate = tgi.xstate;
        reach = false;
    }

    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                                   si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (!validMotion)
    {
        si_->freeState(root_motion->state);
        delete root_motion;
        return TRAPPED;
    }

    /* Cost resampling for new vertices */
    /* Don't bother cost resampling if we don't have a meaningful cost range yet */
    if (!rmotion->connecting && bestCost_.value() < std::numeric_limits<double>::infinity())
    {
        si_->copyState(rmotion->state, dstate);
        g_hat = tgi.start ? si_->distance(dstate, startState) : si_->distance(dstate, goalState);

        int remaining_resample_attempts = maxResampleAttempts_;
        while (validMotion && remaining_resample_attempts > 0)
        {
            remaining_resample_attempts--;

            auto new_cost = si_->distance(nmotion->state, dstate) + nmotion->cost;
            double c_range = new_cost - g_hat;
            double cost_sample = rng_.uniformReal(0, 1);
            double c_rand = (cost_sample * c_range) + g_hat;
            rmotion->cost = c_rand;

            auto rootDist = tree->getDistanceFunction()(rmotion, root_motion);
            Motion *n_nmotion = findNeighbour(rmotion, rootDist, tree);

            if (new_cost <= si_->distance(n_nmotion->state, dstate) + n_nmotion->cost || c_range == 0)
            {
                validMotion = false;
            }
            else
            {
                validMotion = si_->checkMotion(n_nmotion->state, dstate);

                if (validMotion)
                {
                    new_cost = si_->distance(n_nmotion->state, dstate) + n_nmotion->cost;
                    nmotion = n_nmotion;
                    reach = true;
                }
            }
        }
    }

    auto *motion = new Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nmotion;
    motion->root = nmotion->root;
    motion->cost = si_->distance(nmotion->state, motion->state) + nmotion->cost;
    tree->add(motion);

    tgi.xmotion = motion;

    si_->freeState(root_motion->state);
    delete root_motion;
    return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::geometric::AOXRRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    _ptc = ptc;

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    auto *cmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool solved = false;
    GrowState gs;

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->cost = 0;
        tStart_->add(motion);
        startState = motion->state;

        /* Straight line check for first search loop
           Nested here for PDT visualize, which calls solve for each iteration */
        si_->copyState(rstate, startState);
        gs = REACHED;
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
    {
        /* We are using informed sampling, this can end-up reverting to rejection sampling in some cases */
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        sampler_ = opt_->allocInformedStateSampler(pdef_, 100);
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()), bestCost_);

    /* Planner may restart early if it fails to find a solution and start the search anew */
    while (!ptc && !internalResetCondition())
    {
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        TreeData &otherTree = !startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;        

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->cost = 0;

                /* Straight line check for start of planning
                   Nested here for PDT visualize, which calls solve for each iteration */
                if (tGoal_->size() == 0)
                {
                    tgi.xmotion = motion;
                }

                tGoal_->add(motion);
                goalState = motion->state;
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        if (gs != TRAPPED)
        {
            /* remember which motion was added in the last grow */
            Motion *addedMotion = tgi.xmotion;

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            /* Switch tree to try and connect from other side */
            tgi.start = !tgi.start;

            si_->copyState(cmotion->state, rmotion->state);
            cmotion->cost = bestCost_.value() - tgi.xmotion->cost;
            cmotion->connecting = true;

            /* if initial progress cannot be done from the otherTree, restore tgi.start */
            GrowState gsc = growTree(otherTree, tgi, cmotion);
            if (gsc == TRAPPED)
            {
                /* Switch tree back if connect from other side failed */
                tgi.start = !tgi.start;
            }

            /* Keep trying to connect until we reach the other tree or fail to advance */
            while (gsc == ADVANCED)
            {
                gsc = growTree(otherTree, tgi, cmotion);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                /* it must be the case that either the start tree or the goal tree has made some progress
                   so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                   on the solution path */
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
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
                foundPath = path;

                solved = true;

                OMPL_INFORM("%s: Found a solution of cost %.5f", getName().c_str(), path->length());
                break;
            }
            else
            {
                /* We didn't reach the goal, but if we were extending the start
                   tree, then we can mark/improve the approximate path so far. */
                if (tgi.start)
                {
                    /* We were working from the startTree. */
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

        float asize = tree->size();
        float bsize = otherTree->size();
        float ratio = std::abs(asize - bsize) / asize;
        
        /* Swap trees if our balanced RRTC condition is met */
        if (ratio < 1.f)
        {
            startTree_ = !startTree_;
        }
        tgi.start = startTree_;

        /* Sample random states until we find a valid and useful sample */
        bool placed_sampled = false;
        double c_rand;
        double c_range;
        do {
            placed_sampled = sampler_->sampleUniform(rstate, bestCost_);
            sampleAttempts ++;
            
            double cost_sample = rng_.uniformReal(0, 1);
            auto g_hat = tgi.start ? si_->distance(rmotion->state, startState) : si_->distance(rmotion->state, goalState);
            auto h_hat = !tgi.start ? si_->distance(rmotion->state, startState) : si_->distance(rmotion->state, goalState);
            auto f_hat = g_hat + h_hat;

            c_range = bestCost_.value() - f_hat;
            c_rand = (cost_sample * c_range) + g_hat;

        } while (!ptc && (!placed_sampled || c_range < 0));
        rmotion->cost = c_rand;

        /* Extend towards our new sample */
        /* I can't change references when I swap the trees, so this will have to do */
        TreeData &temp_tree = startTree_ ? tStart_ : tGoal_;
        gs = growTree(temp_tree, tgi, rmotion);
    }

    /* Cleanup our memory */
    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;
    delete cmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}
