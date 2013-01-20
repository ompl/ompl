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

#include "ompl/geometric/GeometricOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"

bool ompl::geometric::BoundedAccumulativeOptimizationObjective::isSatisfied(const base::Cost* totalObjectiveCost) const
{    
    return totalObjectiveCost->as<CostType>()->getValue() <= maximumUpperBound_;
}

void ompl::geometric::BoundedAccumulativeOptimizationObjective::getCost(const base::PathPtr& path, base::Cost* cost) const
{
    // Cast path down to a PathGeometric
    boost::shared_ptr<PathGeometric> pathGeom = boost::dynamic_pointer_cast<PathGeometric>(path);

    // Give up if this isn't a PathGeometric or if the path is empty.
    if (!pathGeom)
	OMPL_ERROR("Could not cast PathPtr to PathGeometricPtr");
    else
    {
	unsigned numStates = pathGeom->getStateCount();
	if (numStates == 0)
	    OMPL_ERROR("Cannot compute cost of an empty path.");
	else
	{
	    // Compute path cost by accumulating the cost along the path
	    base::Cost* incCost = allocCost();
	    getInitialCost(pathGeom->getState(0), cost);
	    for (unsigned i = 1; i < numStates; ++i)
	    {
		base::State* s1 = pathGeom->getState(i-1);
		base::State* s2 = pathGeom->getState(i);
		getIncrementalCost(s1, s2, incCost);
		combineObjectiveCosts(cost, incCost, cost);
	    }
	}
    }
}

bool ompl::geometric::BoundedAccumulativeOptimizationObjective::compareCost(const base::Cost* c1, const base::Cost* c2) const
{    
    return c1->as<CostType>()->getValue() < c2->as<CostType>()->getValue();
}

bool ompl::geometric::BoundedAccumulativeOptimizationObjective::isSymmetric(void) const
{
    return si_->getStateSpace()->hasSymmetricInterpolate();
}

// double ompl::geometric::BoundedAccumulativeOptimizationObjective::getTerminalCost(const State *s) const
// {
//   return 0.0;
// }

ompl::geometric::PathLengthOptimizationObjective::PathLengthOptimizationObjective(const base::SpaceInformationPtr &si, double maximumPathLength) : BoundedAccumulativeOptimizationObjective(si, maximumPathLength)
{
    description_ = "Path Length";
}

void ompl::geometric::PathLengthOptimizationObjective::getIncrementalCost(const base::State *s1, const base::State *s2, base::Cost* cost) const
{
    cost->as<CostType>()->value_ = si_->distance(s1,s2);
}

void ompl::geometric::PathLengthOptimizationObjective::combineObjectiveCosts(const base::Cost* c1, const base::Cost* c2, base::Cost* cost) const
{
    cost->as<CostType>()->value_ = 
	c1->as<CostType>()->getValue() +
	c2->as<CostType>()->getValue();
}

void ompl::geometric::PathLengthOptimizationObjective::getInitialCost(const base::State* s, base::Cost* cost) const
{
    cost->as<CostType>()->value_ = 0.0;
}

ompl::base::Cost* ompl::geometric::PathLengthOptimizationObjective::allocCost(void) const
{
    return new CostType;
}

void ompl::geometric::PathLengthOptimizationObjective::copyCost(base::Cost* dest, const base::Cost* src) const
{
    dest->as<CostType>()->value_ = src->as<CostType>()->getValue();
}

void ompl::geometric::PathLengthOptimizationObjective::freeCost(base::Cost* cost) const
{
    delete cost->as<CostType>();
}

ompl::geometric::StateCostOptimizationObjective::StateCostOptimizationObjective(const base::SpaceInformationPtr &si, double maximumPathLength) : BoundedAccumulativeOptimizationObjective(si, maximumPathLength)
{
    description_ = "State Cost";
}

void ompl::geometric::StateCostOptimizationObjective::getIncrementalCost(const base::State *s1, const base::State *s2, base::Cost* cost) const
{
    double c;
    std::pair<double, double> dummy;
    si_->getMotionValidator()->computeMotionCost(s1, s2, c, dummy);
    cost->as<CostType>()->value_ = c;
}

void ompl::geometric::StateCostOptimizationObjective::combineObjectiveCosts(const base::Cost* c1, const base::Cost* c2, base::Cost* cost) const
{
    cost->as<CostType>()->value_ = 
	c1->as<CostType>()->getValue() +
	c2->as<CostType>()->getValue();
}

void ompl::geometric::StateCostOptimizationObjective::getInitialCost(const base::State* s, base::Cost* cost) const
{
    cost->as<CostType>()->value_ = si_->getStateValidityChecker()->cost(s);
}

ompl::base::Cost* ompl::geometric::StateCostOptimizationObjective::allocCost(void) const
{
    return new CostType;
}

void ompl::geometric::StateCostOptimizationObjective::copyCost(base::Cost* dest, const base::Cost* src) const
{
    dest->as<CostType>()->value_ = src->as<CostType>()->getValue();
}

void ompl::geometric::StateCostOptimizationObjective::freeCost(base::Cost* cost) const
{
    delete cost->as<CostType>();
}
