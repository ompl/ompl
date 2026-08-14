/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Worcester Polytechnic Institute ELPIS Lab
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
 *   * Neither the name of the Worcester Polytechnic Institute nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Zhuoyun Zhong */

#include "ompl/control/planners/rrt/AORRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Console.h"
#include <limits>
#include <vector>

ompl::control::AORRT::AORRT(const SpaceInformationPtr &si) : base::Planner(si, "AORRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &AORRT::setGoalBias, &AORRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &AORRT::setIntermediateStates,
                                &AORRT::getIntermediateStates, "0,1");
}

ompl::control::AORRT::~AORRT()
{
    freeMemory();
}

void ompl::control::AORRT::setup()
{
    base::Planner::setup();

    /* Nearest neighbor - begin with regular distance function */
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    /* Optimization objective */
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
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
        OMPL_WARN("%s: problem definition is not set, defer initialization", getName().c_str());
    }
}

void ompl::control::AORRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::AORRT::freeMemory()
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
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::AORRT::solve(const base::PlannerTerminationCondition &ptc)
{
    /* Solve for the first time to find a initial solution
       This should behave similarly as vanila RRT with regular distance function */
    auto solve_status = solveOnce(ptc);
    if (solve_status != base::PlannerStatus::EXACT_SOLUTION)
    {
        if (prevSolution_)
        {
            pdef_->addSolutionPath(prevSolution_, prevApproximate_, prevApproximateDif_, getName());
        }
        OMPL_INFORM("%s: Terminated before finding the first solution", getName().c_str());
        return solve_status;
    }

    prevExactSolution_ = prevSolution_;
    OMPL_INFORM("Found solution with cost %.5f", lastGoalMotion_->totalCost.value());

    /* Solve iteratively to find asymptotically better solutions
       This part will be performed in augmented cost-state space */
    while (!ptc)
    {
        /* Tighten cost bound and attempt to improve;
           Prune nodes in tree that exceeds the new cost bound;
           Also change distance function to augmented space */
        setBestCost(lastGoalMotion_->totalCost);

        solve_status = solveOnce(ptc);
        if (solve_status == base::PlannerStatus::EXACT_SOLUTION)
        {
            prevExactSolution_ = prevSolution_;
            OMPL_INFORM("Found solution with cost %.5f", lastGoalMotion_->totalCost.value());
        }
    }

    pdef_->addSolutionPath(prevExactSolution_, false, prevApproximateDif_, getName());
    return base::PlannerStatus::EXACT_SOLUTION;
}

double ompl::control::AORRT::distanceFunction(const Motion *a, const Motion *b) const
{
    return si_->distance(a->state, b->state);
}

double ompl::control::AORRT::costDistanceFunction(const Motion *a, const Motion *b) const
{
    // Distance between motions in state-cost space
    auto spaceDiff = si_->distance(a->state, b->state);
    auto costDiff = opt_->subtractCosts(a->cost, b->cost).value();

    auto dist = sqrt(pow(spaceDiff, 2) + costWeight_ * pow(costDiff, 2));
    return dist;
}

ompl::base::PlannerStatus ompl::control::AORRT::solveOnce(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        motion->cost = opt_->identityCost();
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    // OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* sample cost */
        double c_rand;
        if (std::isfinite(bestCost_.value()))
        {
            c_rand = rng_.uniformReal(0.0, bestCost_.value());
        }
        /* For the first time, bestCost is infinite, simply provide a constant cost */
        else
        {
            c_rand = 1.0;
        }
        rmotion->cost = base::Cost(c_rand);

        /* find closest valid state in the tree (that satisfies the cost bound) */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* Compute cost */
                    auto costMotion = opt_->controlMotionCost(lastmotion->state, rctrl, 1, pstates[p]);
                    auto newCost = opt_->combineCosts(lastmotion->cost, costMotion);
                    /* Check cost bound
                    This check can be skipped if bestCost_ is infinite (first solving attempt) */
                    if (std::isfinite(bestCost_.value()))
                    {
                        auto heuristicCost = opt_->costToGo(rmotion->state, goal_s);
                        if (!opt_->isCostBetterThan(opt_->combineCosts(newCost, heuristicCost), bestCost_))
                        {
                            break;
                        }
                    }

                    /* create a motion */
                    auto *motion = new Motion(siC_);
                    motion->state = pstates[p];
                    motion->cost = newCost;

                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        /* add terminal cost in the end */
                        auto terminalCost = opt_->terminalCost(motion->state);
                        auto totalCost = opt_->combineCosts(motion->cost, terminalCost);
                        if (!std::isfinite(bestCost_.value()) || opt_->isCostBetterThan(totalCost, bestCost_))
                        {
                            motion->totalCost = totalCost;
                            approxdif = dist;
                            solution = motion;
                            break;
                        }
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* Compute cost */
                auto costMotion = opt_->controlMotionCost(nmotion->state, rctrl, cd, rmotion->state);
                auto newCost = opt_->combineCosts(nmotion->cost, costMotion);
                /* Check cost bound
                   This check can be skipped if bestCost_ is infinite (first solving attempt) */
                if (std::isfinite(bestCost_.value()))
                {
                    auto heuristicCost = opt_->costToGo(rmotion->state, goal_s);
                    if (!opt_->isCostBetterThan(opt_->combineCosts(newCost, heuristicCost), bestCost_))
                    {
                        continue;
                    }
                }

                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;
                motion->cost = newCost;
                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    /* add terminal cost in the end */
                    auto terminalCost = opt_->terminalCost(motion->state);
                    auto totalCost = opt_->combineCosts(motion->cost, terminalCost);
                    if (!std::isfinite(bestCost_.value()) || opt_->isCostBetterThan(totalCost, bestCost_))
                    {
                        motion->totalCost = totalCost;
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
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

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        // pdef_->addSolutionPath(path, approximate, approxdif, getName());
        prevSolution_ = path;
    }
    prevApproximate_ = approximate;
    prevApproximateDif_ = approxdif;

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    // OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    return {solved, approximate};
}

void ompl::control::AORRT::setBestCost(base::Cost cost)
{
    if (!(opt_ && std::isfinite(cost.value()) && cost.value() > 0.0))
    {
        OMPL_WARN("%s: Optimization objective not set or given invalid cost value. Ignoring.", getName().c_str());
        return;
    }

    /* Prune NN - use the previous best cost so the goal would not be pruned */
    pruneNN(cost);

    /* Set a tighten cost bound for next round */
    bestCost_ = base::Cost(cost.value() - std::numeric_limits<double>::epsilon());
    opt_->setCostThreshold(bestCost_);
}

void ompl::control::AORRT::pruneNN(base::Cost costBound)
{
    if (!(nn_ && nn_->size() > 0))
    {
        OMPL_INFORM("%s: No Nearest Neighbors to prune", getName().c_str());
        return;
    }

    std::vector<Motion *> motions;
    nn_->list(motions);

    /* Assuming edge cost is nonnegative, so that this pruning will ensure
       the tree structure is kept */
    std::vector<Motion *> toKeep;
    std::size_t removed = 0;
    bool goal_pruned = false;
    for (auto *m : motions)
    {
        if (opt_->isCostBetterThan(m->cost, costBound) || opt_->isCostEquivalentTo(m->cost, costBound))
        {
            toKeep.push_back(m);
        }
        else
        {
            if (m == lastGoalMotion_)
            {
                goal_pruned = true;
            }
            // free memory for pruned node
            if (m->state)
            {
                si_->freeState(m->state);
            }
            if (m->control)
            {
                siC_->freeControl(m->control);
            }
            delete m;
            ++removed;
        }
    }

    /* Rebuild a fresh NN with only nodes whose cumulative cost is <= costBound */
    nn_->clear();
    /* When pruning is performed, it should mean that a initial best cost is found
       Turns to cost distance function for nearest neighbors */
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return costDistanceFunction(a, b); });
    nn_->add(toKeep);

    if (goal_pruned)
    {
        lastGoalMotion_ = nullptr;
    }

    OMPL_INFORM("%s: Pruned %zu nodes above cost bound %.5f; %u remain.", getName().c_str(), removed, costBound.value(),
                (unsigned)nn_->size());
}

void ompl::control::AORRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
