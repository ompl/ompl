/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Authors: Ioan Sucan, Luis G. Torres */

#include "ompl/base/objectives/StateCostOptimizationObjective.h"

ompl::base::StateCostOptimizationObjective::StateCostOptimizationObjective(const SpaceInformationPtr &si, double maxTotalCost) : 
    AccumulativeOptimizationObjective(si),
    maxTotalCost_(maxTotalCost)
{
    description_ = "State Cost";
}

bool ompl::base::StateCostOptimizationObjective::isSatisfied(const Cost* cost) const
{
    return (cost->as<CostType>()->getValue() <= maxTotalCost_);
}
bool ompl::base::StateCostOptimizationObjective::compareCost(const Cost* c1, const Cost* c2) const
{
    return (c1->as<CostType>()->getValue() < c2->as<CostType>()->getValue());
}

void ompl::base::StateCostOptimizationObjective::getIncrementalCost(const State *s1, const State *s2, Cost* cost) const
{
    double c;
    std::pair<double, double> dummy;
    si_->getMotionValidator()->computeMotionCost(s1, s2, c, dummy);
    cost->as<CostType>()->setValue(c);
}

void ompl::base::StateCostOptimizationObjective::combineObjectiveCosts(const Cost* c1, const Cost* c2, Cost* cost) const
{
    cost->as<CostType>()->setValue(c1->as<CostType>()->getValue() +
				   c2->as<CostType>()->getValue());
}

void ompl::base::StateCostOptimizationObjective::getInitialCost(const State* s, Cost* cost) const
{
    cost->as<CostType>()->setValue(si_->getStateValidityChecker()->cost(s));
}

ompl::base::Cost* ompl::base::StateCostOptimizationObjective::allocCost(void) const
{
    return new CostType;
}

void ompl::base::StateCostOptimizationObjective::copyCost(Cost* dest, const Cost* src) const
{
    dest->as<CostType>()->setValue(src->as<CostType>()->getValue());
}

void ompl::base::StateCostOptimizationObjective::freeCost(Cost* cost) const
{
    delete cost->as<CostType>();
}
