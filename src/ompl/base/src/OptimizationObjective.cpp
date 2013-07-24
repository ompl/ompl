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

ompl::base::Cost ompl::base::OptimizationObjective::getCost(const Path &path) const
{
    // Cast path down to a PathGeometric
    const geometric::PathGeometric *pathGeom = dynamic_cast<const geometric::PathGeometric*>(&path);

    // Give up if this isn't a PathGeometric or if the path is empty.
    if (!pathGeom)
    {
	OMPL_ERROR("Could not cast Path* to PathGeometric* in ompl::base::OptimizationObjective::getCost()");
        return this->identityCost();
    }
    else
    {
        std::size_t numStates = pathGeom->getStateCount();
	if (numStates == 0)
        {
	    OMPL_ERROR("Cannot compute cost of an empty path.");
            return this->identityCost();
        }
	else
	{
	    // Compute path cost by accumulating the cost along the path
            Cost cost(this->identityCost());
	    for (std::size_t i = 1; i < numStates; ++i)
	    {
		const State *s1 = pathGeom->getState(i-1);
		const State *s2 = pathGeom->getState(i);
                cost = this->combineCosts(cost, this->motionCost(s1, s2));
	    }

            return cost;
	}
    }
}

ompl::base::Cost ompl::base::OptimizationObjective::averageStateCost(unsigned int numStates) const
{
    StateSamplerPtr ss = si_->allocStateSampler();
    State *state = si_->allocState();
    Cost totalCost = 0.0;

    for (unsigned int i = 0 ; i < numStates ; ++i)
    {
        ss->sampleUniform(state);
        this->combineCosts(totalCost, this->stateCost(state));
    }

    si_->freeState(state);

    return Cost(totalCost.v / (double)numStates);
}

ompl::base::Cost ompl::base::StateCostIntegralObjective::motionCost(const State *s1, 
                                                                    const State *s2) const
{
    return Cost(0.5 * si_->distance(s1, s2) * (this->stateCost(s1).v + this->stateCost(s2).v));
}

double ompl::base::MechanicalWorkOptimizationObjective::getPathLengthWeight(void) const
{
    return pathLengthWeight_;
}

void ompl::base::MechanicalWorkOptimizationObjective::setPathLengthWeight(double weight)
{
    pathLengthWeight_ = weight;
}

ompl::base::Cost ompl::base::MechanicalWorkOptimizationObjective::motionCost(const State *s1, 
                                                                             const State *s2) const
{
    // Only accrue positive changes in cost
    double positiveCostAccrued = std::max(stateCost(s2).v - stateCost(s1).v, 0.0);
    return Cost(positiveCostAccrued + pathLengthWeight_*si_->distance(s1,s2));
}

ompl::base::Cost ompl::base::MinimaxObjective::motionCost(const State *s1, const State *s2) const
{
    Cost worstCost = this->identityCost();

    // \TODO shouldn't we be able to use some method in SpaceInformation
    // for this instead?
    int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
  
    if (nd > 1)
    {
        State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
        {
            si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test);
            Cost testStateCost = this->stateCost(test);
            if (this->isCostBetterThan(worstCost, testStateCost))
                worstCost = testStateCost;
        }
        si_->freeState(test);
    }

    // Lastly, check s2
    Cost lastCost = this->stateCost(s2);
    if (this->isCostBetterThan(worstCost, lastCost))
        worstCost = lastCost; 

    return worstCost;
}

ompl::base::Cost ompl::base::MinimaxObjective::combineCosts(Cost c1, Cost c2) const
{
    if (this->isCostBetterThan(c1, c2))
        return c2;
    else
        return c1;
}

ompl::base::Cost ompl::base::MaximizeMinClearanceObjective::stateCost(const State* s) const
{
    return Cost(si_->getStateValidityChecker()->clearance(s));
}

bool ompl::base::MaximizeMinClearanceObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.v > c2.v;
}

ompl::base::Cost ompl::base::MaximizeMinClearanceObjective::identityCost(void) const
{
    return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::MaximizeMinClearanceObjective::infiniteCost(void) const
{
    return Cost(-std::numeric_limits<double>::infinity());
}
