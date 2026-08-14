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

/* Authors: Zhuoyun Zhong */
/* Adapted from: ompl/control/planners/sst/src/SST.cpp by Zakary Littlefield */

#include "ompl/control/planners/sst/SSTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/OptimizationObjective.h"

#include <cmath>
#include <limits>

ompl::control::SSTstar::SSTstar(const SpaceInformationPtr &si) : SST(si)
{
    setName("SSTstar");
    Planner::declareParam<double>("initial_iteration", this, &SSTstar::setInitialIteration,
                                  &SSTstar::getInitialIteration, "10:10:100000");
    Planner::declareParam<double>("shrink_factor", this, &SSTstar::setShrinkFactor, &SSTstar::getShrinkFactor,
                                  "0.:.01:1.");
}

ompl::base::PlannerStatus ompl::control::SSTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (pruningRadius_ > selectionRadius_)
    {
        pruningRadius_ = selectionRadius_ * (1.0 - 1e-3);
        OMPL_WARN("%s: Pruning radius is greater than selection radius. Setting pruning radius to %f",
                  getName().c_str(), pruningRadius_);
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state_, st);
        siC_->nullControl(motion->control_);
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
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure\n", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state_;
    Control *rctrl = rmotion->control_;

    unsigned iterations = 0;
    unsigned outerCount = 0;
    unsigned long innerIteration = initialN_;
    const double stateDimension = static_cast<double>(si_->getStateDimension());
    const double controlDimension = static_cast<double>(siC_->getControlSpace()->getDimension());

    while (ptc == false)
    {
        unsigned long innerCount = 0;
        while (innerCount < innerIteration && ptc == false)
        {
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                goal_s->sampleGoal(rstate);
            else
                sampler_->sampleUniform(rstate);

            Motion *nmotion = selectNode(rmotion);
            controlSampler_->sample(rctrl);
            unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
            unsigned int propCd = siC_->propagateWhileValid(nmotion->state_, rctrl, cd, rstate);

            if (propCd == cd)
            {
                base::Cost incCost = opt_->controlMotionCost(nmotion->state_, rctrl, cd, rstate);
                base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);
                Witness *closestWitness = findClosestWitness(rmotion);

                if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
                {
                    Motion *oldRep = closestWitness->rep_;
                    auto *motion = new Motion(siC_);
                    motion->accCost_ = cost;
                    si_->copyState(motion->state_, rmotion->state_);
                    siC_->copyControl(motion->control_, rctrl);
                    motion->steps_ = cd;
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

                        for (auto &state : prevSolution_)
                            if (state)
                                si_->freeState(state);
                        prevSolution_.clear();
                        for (auto &control : prevSolutionControls_)
                            if (control)
                                siC_->freeControl(control);
                        prevSolutionControls_.clear();
                        prevSolutionSteps_.clear();

                        Motion *solTrav = solution;
                        while (solTrav->parent_ != nullptr)
                        {
                            prevSolution_.push_back(si_->cloneState(solTrav->state_));
                            prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                            prevSolutionSteps_.push_back(solTrav->steps_);
                            solTrav = solTrav->parent_;
                        }
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        prevSolutionCost_ = solution->accCost_;

                        OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
                        if (intermediateSolutionCallback)
                        {
                            std::vector<const base::State *> prevSolutionConst(prevSolution_.begin(), prevSolution_.end());
                            intermediateSolutionCallback(this, prevSolutionConst, prevSolutionCost_);
                        }
                        sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                        if (sufficientlyShort)
                            break;
                    }
                    if (solution == nullptr && dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;

                        for (auto &state : prevSolution_)
                            if (state)
                                si_->freeState(state);
                        prevSolution_.clear();
                        for (auto &control : prevSolutionControls_)
                            if (control)
                                siC_->freeControl(control);
                        prevSolutionControls_.clear();
                        prevSolutionSteps_.clear();

                        Motion *solTrav = approxsol;
                        while (solTrav->parent_ != nullptr)
                        {
                            prevSolution_.push_back(si_->cloneState(solTrav->state_));
                            prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                            prevSolutionSteps_.push_back(solTrav->steps_);
                            solTrav = solTrav->parent_;
                        }
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                    }

                    if (oldRep != rmotion)
                    {
                        while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                        {
                            oldRep->inactive_ = true;
                            nn_->remove(oldRep);

                            if (oldRep->state_)
                                si_->freeState(oldRep->state_);
                            if (oldRep->control_)
                                siC_->freeControl(oldRep->control_);

                            oldRep->state_ = nullptr;
                            oldRep->control_ = nullptr;
                            oldRep->parent_->numChildren_--;
                            Motion *oldRepParent = oldRep->parent_;
                            delete oldRep;
                            oldRep = oldRepParent;
                        }
                    }
                }
            }

            ++iterations;
            ++innerCount;
        }

        selectionRadius_ *= shrinkFactor_;
        pruningRadius_ *= shrinkFactor_;
        if (selectionRadius_ < 1e-9)
            selectionRadius_ = 1e-9;
        if (pruningRadius_ < 1e-9)
            pruningRadius_ = 1e-9;

        ++outerCount;
        const double batch = static_cast<double>(outerCount);
        const double multiplier = (1.0 + std::log(batch)) *
                                  std::pow(1.0 / shrinkFactor_, (stateDimension + controlDimension + 1.0) * batch);
        innerIteration = static_cast<unsigned long>(std::ceil(multiplier * static_cast<double>(initialN_)));
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
        auto path(std::make_shared<PathControl>(si_));
        for (int i = prevSolution_.size() - 1; i >= 1; --i)
            path->append(prevSolution_[i], prevSolutionControls_[i - 1],
                         prevSolutionSteps_[i - 1] * siC_->getPropagationStepSize());
        path->append(prevSolution_[0]);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);
    return {solved, approximate};
}