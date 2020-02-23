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

/* Author: Luis G. Torres, Ioan Sucan, Jonathan Gammell */

#include "ompl/base/OptimizationObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/base/goals/GoalRegion.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include <limits>
// For std::make_shared
#include <memory>
#include <utility>

ompl::base::OptimizationObjective::OptimizationObjective(SpaceInformationPtr si) : si_(std::move(si)), threshold_(0.0)
{
}

const std::string &ompl::base::OptimizationObjective::getDescription() const
{
    return description_;
}

bool ompl::base::OptimizationObjective::isSatisfied(Cost c) const
{
    return isCostBetterThan(c, threshold_);
}

ompl::base::Cost ompl::base::OptimizationObjective::getCostThreshold() const
{
    return threshold_;
}

void ompl::base::OptimizationObjective::setCostThreshold(Cost c)
{
    threshold_ = c;
}

bool ompl::base::OptimizationObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.value() < c2.value();
}

bool ompl::base::OptimizationObjective::isCostEquivalentTo(Cost c1, Cost c2) const
{
    // If c1 is not better than c2, and c2 is not better than c1, then they are equal
    return !isCostBetterThan(c1, c2) && !isCostBetterThan(c2, c1);
}

bool ompl::base::OptimizationObjective::isFinite(Cost cost) const
{
    return isCostBetterThan(cost, infiniteCost());
}

ompl::base::Cost ompl::base::OptimizationObjective::betterCost(Cost c1, Cost c2) const
{
    return isCostBetterThan(c1, c2) ? c1 : c2;
}

ompl::base::Cost ompl::base::OptimizationObjective::combineCosts(Cost c1, Cost c2) const
{
    return Cost(c1.value() + c2.value());
}

ompl::base::Cost ompl::base::OptimizationObjective::identityCost() const
{
    return Cost(0.0);
}

ompl::base::Cost ompl::base::OptimizationObjective::infiniteCost() const
{
    return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::OptimizationObjective::initialCost(const State *) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::OptimizationObjective::terminalCost(const State *) const
{
    return identityCost();
}

bool ompl::base::OptimizationObjective::isSymmetric() const
{
    return si_->getStateSpace()->hasSymmetricInterpolate();
}

ompl::base::Cost ompl::base::OptimizationObjective::averageStateCost(unsigned int numStates) const
{
    StateSamplerPtr ss = si_->allocStateSampler();
    State *state = si_->allocState();
    Cost totalCost(identityCost());

    for (unsigned int i = 0; i < numStates; ++i)
    {
        ss->sampleUniform(state);
        totalCost = combineCosts(totalCost, stateCost(state));
    }

    si_->freeState(state);

    return Cost(totalCost.value() / (double)numStates);
}

void ompl::base::OptimizationObjective::setCostToGoHeuristic(const CostToGoHeuristic &costToGo)
{
    costToGoFn_ = costToGo;
}

bool ompl::base::OptimizationObjective::hasCostToGoHeuristic() const
{
    return static_cast<bool>(costToGoFn_);
}

ompl::base::Cost ompl::base::OptimizationObjective::costToGo(const State *state, const Goal *goal) const
{
    if (hasCostToGoHeuristic())
        return costToGoFn_(state, goal);

    return identityCost();  // assumes that identity < all costs
}

ompl::base::Cost ompl::base::OptimizationObjective::motionCostHeuristic(const State *, const State *) const
{
    return identityCost();  // assumes that identity < all costs
}

ompl::base::Cost ompl::base::OptimizationObjective::motionCostBestEstimate(const State *, const State *) const
{
    return identityCost();  // assumes that identity < all costs
}

const ompl::base::SpaceInformationPtr &ompl::base::OptimizationObjective::getSpaceInformation() const
{
    return si_;
}

ompl::base::InformedSamplerPtr ompl::base::OptimizationObjective::allocInformedStateSampler(
    const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
{
    OMPL_INFORM("%s: No direct informed sampling scheme is defined, defaulting to rejection sampling.",
                description_.c_str());
    return std::make_shared<RejectionInfSampler>(probDefn, maxNumberCalls);
}

void ompl::base::OptimizationObjective::print(std::ostream &out) const
{
    out << "Optimization Objective: " << description_ << " @" << this << std::endl;
    out << "Optimization Threshold: " << threshold_ << std::endl;
}

ompl::base::Cost ompl::base::goalRegionCostToGo(const State *state, const Goal *goal)
{
    const auto *goalRegion = goal->as<GoalRegion>();

    // Ensures that all states within the goal region's threshold to
    // have a cost-to-go of exactly zero.
    return Cost(std::max(goalRegion->distanceGoal(state) - goalRegion->getThreshold(), 0.0));
}

ompl::base::MultiOptimizationObjective::MultiOptimizationObjective(const SpaceInformationPtr &si)
  : OptimizationObjective(si), locked_(false)
{
}

ompl::base::MultiOptimizationObjective::Component::Component(OptimizationObjectivePtr obj, double weight)
  : objective(std::move(obj)), weight(weight)
{
}

void ompl::base::MultiOptimizationObjective::addObjective(const OptimizationObjectivePtr &objective, double weight)
{
    if (locked_)
    {
        throw Exception("This optimization objective is locked. No further objectives can be added.");
    }
    else
        components_.emplace_back(objective, weight);
}

std::size_t ompl::base::MultiOptimizationObjective::getObjectiveCount() const
{
    return components_.size();
}

const ompl::base::OptimizationObjectivePtr &ompl::base::MultiOptimizationObjective::getObjective(unsigned int idx) const
{
    if (components_.size() > idx)
        return components_[idx].objective;
    throw Exception("Objective index does not exist.");
}

double ompl::base::MultiOptimizationObjective::getObjectiveWeight(unsigned int idx) const
{
    if (components_.size() > idx)
        return components_[idx].weight;
    throw Exception("Objective index does not exist.");
}

void ompl::base::MultiOptimizationObjective::setObjectiveWeight(unsigned int idx, double weight)
{
    if (components_.size() > idx)
        components_[idx].weight = weight;
    else
        throw Exception("Objecitve index does not exist.");
}

void ompl::base::MultiOptimizationObjective::lock()
{
    locked_ = true;
}

bool ompl::base::MultiOptimizationObjective::isLocked() const
{
    return locked_;
}

ompl::base::Cost ompl::base::MultiOptimizationObjective::stateCost(const State *s) const
{
    Cost c = identityCost();
    for (const auto &component : components_)
    {
        c = Cost(c.value() + component.weight * (component.objective->stateCost(s).value()));
    }

    return c;
}

ompl::base::Cost ompl::base::MultiOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    Cost c = identityCost();
    for (const auto &component : components_)
    {
        c = Cost(c.value() + component.weight * (component.objective->motionCost(s1, s2).value()));
    }

    return c;
}

ompl::base::OptimizationObjectivePtr ompl::base::operator+(const OptimizationObjectivePtr &a,
                                                           const OptimizationObjectivePtr &b)
{
    std::vector<MultiOptimizationObjective::Component> components;

    if (a)
    {
        if (auto *mult = dynamic_cast<MultiOptimizationObjective *>(a.get()))
        {
            for (std::size_t i = 0; i < mult->getObjectiveCount(); ++i)
            {
                components.emplace_back(mult->getObjective(i), mult->getObjectiveWeight(i));
            }
        }
        else
            components.emplace_back(a, 1.0);
    }

    if (b)
    {
        if (auto *mult = dynamic_cast<MultiOptimizationObjective *>(b.get()))
        {
            for (std::size_t i = 0; i < mult->getObjectiveCount(); ++i)
            {
                components.emplace_back(mult->getObjective(i), mult->getObjectiveWeight(i));
            }
        }
        else
            components.emplace_back(b, 1.0);
    }

    auto multObj(std::make_shared<MultiOptimizationObjective>(a->getSpaceInformation()));
    for (const auto &comp : components)
        multObj->addObjective(comp.objective, comp.weight);

    return multObj;
}

ompl::base::OptimizationObjectivePtr ompl::base::operator*(double weight, const OptimizationObjectivePtr &a)
{
    std::vector<MultiOptimizationObjective::Component> components;

    if (a)
    {
        if (auto *mult = dynamic_cast<MultiOptimizationObjective *>(a.get()))
        {
            for (std::size_t i = 0; i < mult->getObjectiveCount(); ++i)
            {
                components.emplace_back(mult->getObjective(i), weight * mult->getObjectiveWeight(i));
            }
        }
        else
            components.emplace_back(a, weight);
    }

    auto multObj(std::make_shared<MultiOptimizationObjective>(a->getSpaceInformation()));
    for (auto const &comp : components)
        multObj->addObjective(comp.objective, comp.weight);

    return multObj;
}

ompl::base::OptimizationObjectivePtr ompl::base::operator*(const OptimizationObjectivePtr &a, double weight)
{
    return weight * a;
}
