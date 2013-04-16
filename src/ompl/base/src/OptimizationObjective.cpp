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

/* Author: Ioan Sucan, Luis G. Torres */

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/tools/config/MagicConstants.h"

void ompl::base::OptimizationObjective::getCost(const Path &path, Cost *cost) const
{
    // Cast path down to a PathGeometric
    const geometric::PathGeometric *pathGeom = dynamic_cast<const geometric::PathGeometric*>(&path);

    // Give up if this isn't a PathGeometric or if the path is empty.
    if (!pathGeom)
	OMPL_ERROR("Could not cast Path* to PathGeometric* in ompl::base::OptimizationObjective::getCost()");
    else
    {
        std::size_t numStates = pathGeom->getStateCount();
	if (numStates == 0)
	    OMPL_ERROR("Cannot compute cost of an empty path.");
	else
	{
	    // Compute path cost by accumulating the cost along the path
	    Cost *incCost = allocCost();
	    getInitialCost(pathGeom->getState(0), cost);
	    for (std::size_t i = 1; i < numStates; ++i)
	    {
		const State *s1 = pathGeom->getState(i-1);
		const State *s2 = pathGeom->getState(i);
		getIncrementalCost(s1, s2, incCost);
		combineObjectiveCosts(cost, incCost, cost);
	    }
	    freeCost(incCost);
	}
    }
}

bool ompl::base::OptimizationObjective::isCostLessThan(const Cost *c1, const Cost *c2) const
{
  return (getCostValue(c1) + magic::BETTER_PATH_COST_MARGIN < getCostValue(c2));
}

double ompl::base::OptimizationObjective::getStateCost(const State *s) const
{
    return 1.0;
}

bool ompl::base::OptimizationObjective::isSymmetric(void) const
{
    return si_->getStateSpace()->hasSymmetricInterpolate();
}

double ompl::base::OptimizationObjective::averageStateCost(unsigned int numStates) const
{
    StateSamplerPtr ss = si_->allocStateSampler();
    State *state = si_->allocState();
    double totalCost = 0.0;

    for (unsigned int i = 0 ; i < numStates ; ++i)
    {
        ss->sampleUniform(state);
        totalCost += getStateCost(state);
    }

    si_->freeState(state);

    return totalCost / (double)numStates;
}

ompl::base::PathIntegralOptimizationObjective::PathIntegralOptimizationObjective(const SpaceInformationPtr &si, double maxPathCost) :
    OptimizationObjective(si)
{
    description_ = "Path Integral";
}

double ompl::base::PathIntegralOptimizationObjective::getCostValue(const Cost *cost) const
{
  return cost->as<CostType>()->value;
}

void ompl::base::PathIntegralOptimizationObjective::getCost(const Path &path, Cost *cost) const
{
    OptimizationObjective::getCost(path, cost);
}

double ompl::base::PathIntegralOptimizationObjective::getCost(const Path &path) const
{
    Cost *cost = allocCost();
    getCost(path, cost);
    double value = cost->as<CostType>()->value;
    freeCost(cost);
    return value;
}

double ompl::base::PathIntegralOptimizationObjective::getMaxPathCost(void) const
{
    return maxPathCost_;
}

void ompl::base::PathIntegralOptimizationObjective::setMaxPathCost(double maxPathCost)
{
    maxPathCost_ = maxPathCost;
}

bool ompl::base::PathIntegralOptimizationObjective::isSatisfied(const Cost *cost) const
{
    return (cost->as<CostType>()->value <= maxPathCost_);
}

void ompl::base::PathIntegralOptimizationObjective::getIncrementalCost(const State *s1, const State *s2, Cost *cost) const
{
    cost->as<CostType>()->value = si_->distance(s1,s2)*(getStateCost(s1) + getStateCost(s2)) / 2.0;
}

void ompl::base::PathIntegralOptimizationObjective::combineObjectiveCosts(const Cost *c1, const Cost *c2, Cost *cost) const
{
    cost->as<CostType>()->value = c1->as<CostType>()->value + c2->as<CostType>()->value;
}

void ompl::base::PathIntegralOptimizationObjective::getInitialCost(const State *s, Cost *cost) const
{
    cost->as<CostType>()->value = 0.0;
}

void ompl::base::PathIntegralOptimizationObjective::getInfiniteCost(Cost *cost) const
{
  cost->as<CostType>()->value = std::numeric_limits<double>::infinity();
}

ompl::base::Cost* ompl::base::PathIntegralOptimizationObjective::allocCost(void) const
{
    return new CostType;
}

void ompl::base::PathIntegralOptimizationObjective::copyCost(Cost *dest, const Cost *src) const
{
    dest->as<CostType>()->value = src->as<CostType>()->value;
}

void ompl::base::PathIntegralOptimizationObjective::freeCost(Cost *cost) const
{
    delete cost->as<CostType>();
}

double ompl::base::MechanicalWorkOptimizationObjective::getPathLengthWeight(void) const
{
    return pathLengthWeight_;
}

void ompl::base::MechanicalWorkOptimizationObjective::setPathLengthWeight(double weight)
{
    pathLengthWeight_ = weight;
}

void ompl::base::MechanicalWorkOptimizationObjective::getIncrementalCost(const State *s1, const State *s2, Cost *cost) const
{
    // Only accrue positive changes in cost
    double positiveCostAccrued = std::max(getStateCost(s2) - getStateCost(s1), 0.0);
    cost->as<CostType>()->value = positiveCostAccrued + pathLengthWeight_*si_->distance(s1,s2);
}
