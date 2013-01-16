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

bool ompl::geometric::BoundedAccumulativeOptimizationObjective::isSatisfied(const base::CostPtr& totalObjectiveCost) const
{    
    return base::Cost::as<CostType>(totalObjectiveCost)->getValue() <= maximumUpperBound_;
}

ompl::base::CostPtr ompl::geometric::BoundedAccumulativeOptimizationObjective::getCost(const base::PathPtr& path) const
{
    base::CostPtr cost;

    // Cast path down to a PathGeometric
    boost::shared_ptr<PathGeometric> pathGeom = boost::dynamic_pointer_cast<PathGeometric>(path);

    if (!pathGeom)
	OMPL_ERROR("Could not cast PathPtr to PathGeometricPtr");
    else
    {
	unsigned numStates = pathGeom->getStateCount();
	if (numStates == 0)
	    OMPL_ERROR("Cannot compute cost of an empty path.");
	else
	{
	    cost = getInitialCost(pathGeom->getState(0));
	    for (unsigned i = 1; i < numStates; ++i)
	    {
		base::State* s1 = pathGeom->getState(i-1);
		base::State* s2 = pathGeom->getState(i);
		cost = combineObjectiveCosts(cost, getIncrementalCost(s1, s2));
	    }
	}
    }

    return cost;
}

bool ompl::geometric::BoundedAccumulativeOptimizationObjective::compareCost(const base::CostPtr& c1, const base::CostPtr& c2) const
{    
    return 
	base::Cost::as<CostType>(c1)->getValue() <
	base::Cost::as<CostType>(c2)->getValue();
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

ompl::base::CostPtr ompl::geometric::PathLengthOptimizationObjective::getIncrementalCost(const base::State *s1, const base::State *s2) const
{
    return base::CostPtr(new CostType(si_->distance(s1,s2)));
}

ompl::base::CostPtr ompl::geometric::PathLengthOptimizationObjective::combineObjectiveCosts(const base::CostPtr& c1, const base::CostPtr& c2) const
{
    return base::CostPtr(new CostType(base::Cost::as<CostType>(c1)->getValue() + 
				      base::Cost::as<CostType>(c2)->getValue()));
}

ompl::base::CostPtr ompl::geometric::PathLengthOptimizationObjective::getInitialCost(const base::State* s) const
{
    return base::CostPtr(new CostType(0.0));
}

ompl::geometric::StateCostOptimizationObjective::StateCostOptimizationObjective(const base::SpaceInformationPtr &si, double maximumPathLength) : BoundedAccumulativeOptimizationObjective(si, maximumPathLength)
{
    description_ = "State Cost";
}

ompl::base::CostPtr ompl::geometric::StateCostOptimizationObjective::getIncrementalCost(const base::State *s1, const base::State *s2) const
{
    double c;
    std::pair<double, double> dummy;
    si_->getMotionValidator()->computeMotionCost(s1, s2, c, dummy);
    return base::CostPtr(new CostType(c));
}

ompl::base::CostPtr ompl::geometric::StateCostOptimizationObjective::combineObjectiveCosts(const base::CostPtr& c1, const base::CostPtr& c2) const
{
    return base::CostPtr(new CostType(base::Cost::as<CostType>(c1)->getValue() + 
				      base::Cost::as<CostType>(c2)->getValue()));
}

ompl::base::CostPtr ompl::geometric::StateCostOptimizationObjective::getInitialCost(const base::State* s) const
{
    return base::CostPtr(new CostType(si_->getStateValidityChecker()->cost(s)));
}
