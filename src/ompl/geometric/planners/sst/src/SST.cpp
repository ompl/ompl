/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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

/* Authors: Zakary Littlefield */

#include "ompl/geometric/planners/sst/SST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::SST::SST(const base::SpaceInformationPtr &si) : base::Planner(si, "SST")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    prevSolution_.clear();

    Planner::declareParam<double>("range", this, &SST::setRange, &SST::getRange, ".1:.1:100");
    Planner::declareParam<double>("goal_bias", this, &SST::setGoalBias, &SST::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("selection_radius", this, &SST::setSelectionRadius, &SST::getSelectionRadius, "0.:.1:"
                                                                                                                "100");
    Planner::declareParam<double>("pruning_radius", this, &SST::setPruningRadius, &SST::getPruningRadius, "0.:.1:100");

    addPlannerProgressProperty("best cost REAL", [this] { return std::to_string(this->prevSolutionCost_.value()); });
}

ompl::geometric::SST::~SST()
{
    freeMemory();
}

void ompl::geometric::SST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    {
                                        return distanceFunction(a, b);
                                    });

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

void ompl::geometric::SST::clear()
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

void ompl::geometric::SST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state_)
                si_->freeState(motion->state_);
            delete motion;
        }
    }
    if (witnesses_)
    {
        std::vector<Motion *> witnesses;
        witnesses_->list(witnesses);
        for (auto &witness : witnesses)
        {
            if (witness->state_)
                si_->freeState(witness->state_);
            delete witness;
        }
    }

    for (auto &i : prevSolution_)
    {
        if (i)
            si_->freeState(i);
    }
    prevSolution_.clear();
}

ompl::geometric::SST::Motion *ompl::geometric::SST::selectNode(ompl::geometric::SST::Motion *sample)
{
    std::vector<Motion *> ret;
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret);
    for (auto &i : ret)
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}

ompl::geometric::SST::Witness *ompl::geometric::SST::findClosestWitness(ompl::geometric::SST::Motion *node)
{
    if (witnesses_->size() > 0)
    {
        auto *closest = static_cast<Witness *>(witnesses_->nearest(node));
        if (distanceFunction(closest, node) > pruningRadius_)
        {
            closest = new Witness(si_);
            closest->linkRep(node);
            si_->copyState(closest->state_, node->state_);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        auto *closest = new Witness(si_);
        closest->linkRep(node);
        si_->copyState(closest->state_, node->state_);
        witnesses_->add(closest);
        return closest;
    }
}

ompl::base::State *ompl::geometric::SST::monteCarloProp(Motion *m)
{
    // sample random point to serve as a direction
    base::State *xstate = si_->allocState();
    sampler_->sampleUniform(xstate);

    // sample length of step from (0 - maxDistance_]
    double step = rng_.uniformReal(0, maxDistance_);

    // take a step of length step towards the random state
    double d = si_->distance(m->state_, xstate);
    si_->getStateSpace()->interpolate(m->state_, xstate, step / d, xstate);
    si_->enforceBounds(xstate);

    return xstate;
}

ompl::base::PlannerStatus ompl::geometric::SST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state_, st);
        nn_->add(motion);
        motion->accCost_ = opt_->identityCost();
        findClosestWitness(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state_;
    base::State *xstate = si_->allocState();

    unsigned iterations = 0;

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        bool attemptToReachGoal = (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample());
        if (attemptToReachGoal)
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = selectNode(rmotion);

        base::State *dstate = rstate;
        double d = si_->distance(nmotion->state_, rstate);

        attemptToReachGoal = rng_.uniform01() < .5;
        if (attemptToReachGoal)
        {
            if (d > maxDistance_)
            {
                si_->getStateSpace()->interpolate(nmotion->state_, rstate, maxDistance_ / d, xstate);
                dstate = xstate;
            }
        }
        else
        {
            dstate = monteCarloProp(nmotion);
        }

        si_->copyState(rstate, dstate);

        if (si_->checkMotion(nmotion->state_, rstate))
        {
            base::Cost incCost = opt_->motionCost(nmotion->state_, rstate);
            base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);
            Witness *closestWitness = findClosestWitness(rmotion);

            if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
            {
                Motion *oldRep = closestWitness->rep_;
                /* create a motion */
                auto *motion = new Motion(si_);
                motion->accCost_ = cost;
                si_->copyState(motion->state_, rstate);

                if (!attemptToReachGoal)
                    si_->freeState(dstate);
                motion->parent_ = nmotion;
                nmotion->numChildren_++;
                closestWitness->linkRep(motion);

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state_, &dist);
                if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
                {
                    approxdif = dist;
                    solution = motion;

                    for (auto &i : prevSolution_)
                        if (i)
                            si_->freeState(i);
                    prevSolution_.clear();
                    Motion *solTrav = solution;
                    while (solTrav != nullptr)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        solTrav = solTrav->parent_;
                    }
                    prevSolutionCost_ = solution->accCost_;

                    OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
                    sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                    if (sufficientlyShort)
                    {
                        break;
                    }
                }
                if (solution == nullptr && dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;

                    for (auto &i : prevSolution_)
                    {
                        if (i)
                            si_->freeState(i);
                    }
                    prevSolution_.clear();
                    Motion *solTrav = approxsol;
                    while (solTrav != nullptr)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        solTrav = solTrav->parent_;
                    }
                }

                if (oldRep != rmotion)
                {
                    while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                    {
                        oldRep->inactive_ = true;
                        nn_->remove(oldRep);

                        if (oldRep->state_)
                            si_->freeState(oldRep->state_);

                        oldRep->state_ = nullptr;
                        oldRep->parent_->numChildren_--;
                        Motion *oldRepParent = oldRep->parent_;
                        delete oldRep;
                        oldRep = oldRepParent;
                    }
                }
            }
        }
        iterations++;
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
        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = prevSolution_.size() - 1; i >= 0; --i)
            path->append(prevSolution_[i]);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    rmotion->state_ = nullptr;
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);

    return {solved, approximate};
}

void ompl::geometric::SST::getPlannerData(base::PlannerData &data) const
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
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto &allMotion : allMotions)
    {
        if (allMotion->getParent() == nullptr)
            data.addStartVertex(base::PlannerDataVertex(allMotion->getState()));
        else
            data.addEdge(base::PlannerDataVertex(allMotion->getParent()->getState()),
                         base::PlannerDataVertex(allMotion->getState()));
    }
}
