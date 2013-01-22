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

#include "ompl/base/objectives/AccumulativeOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"

void ompl::base::AccumulativeOptimizationObjective::getCost(const Path* path, Cost* cost) const
{
    // Cast path down to a PathGeometric
  const geometric::PathGeometric* pathGeom = dynamic_cast<const geometric::PathGeometric*>(path);

    // Give up if this isn't a PathGeometric or if the path is empty.
    if (!pathGeom)
	OMPL_ERROR("Could not cast Path* to PathGeometric* in ompl::base::AccumulativeOptimizationObjective::getCost()");
    else
    {
        std::size_t numStates = pathGeom->getStateCount();
	if (numStates == 0)
	    OMPL_ERROR("Cannot compute cost of an empty path.");
	else
	{
	    // Compute path cost by accumulating the cost along the path
	    Cost* incCost = allocCost();
	    getInitialCost(pathGeom->getState(0), cost);
	    for (std::size_t i = 1; i < numStates; ++i)
	    {
		State* s1 = pathGeom->getState(i-1);
		State* s2 = pathGeom->getState(i);
		getIncrementalCost(s1, s2, incCost);
		combineObjectiveCosts(cost, incCost, cost);
	    }
	    freeCost(incCost);
	}
    }
}

bool ompl::base::AccumulativeOptimizationObjective::isSymmetric(void) const
{
    return si_->getStateSpace()->hasSymmetricInterpolate();
}

