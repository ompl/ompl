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

/* Author: Ioan Sucan */

#include "ompl/base/OptimizationObjective.h"

ompl::base::OptimizationObjective::OptimizationObjective(const SpaceInformationPtr &si) : si_(si)
{
}

double ompl::base::OptimizationObjective::getCost(const PathPtr &path) const
{
    return path->cost(*this);
}

bool ompl::base::BoundedAdditiveOptimizationObjective::isSatisfied(double totalObjectiveCost) const
{
    return totalObjectiveCost <= maximumUpperBound_;
}

double ompl::base::BoundedAdditiveOptimizationObjective::combineObjectiveCosts(double a, double b) const
{
    return a + b;
}

ompl::base::PathLengthOptimizationObjective::PathLengthOptimizationObjective(const SpaceInformationPtr &si, double maximumPathLength) : BoundedAdditiveOptimizationObjective(si, maximumPathLength)
{
    description_ = "Path Length";
}

double ompl::base::PathLengthOptimizationObjective::getIncrementalCost(const State *s1, const State *s2) const
{
    return si_->distance(s1, s2);
}

ompl::base::StateCostOptimizationObjective::StateCostOptimizationObjective(const SpaceInformationPtr &si, double maximumPathLength) : BoundedAdditiveOptimizationObjective(si, maximumPathLength)
{
    description_ = "State Cost";
}

double ompl::base::StateCostOptimizationObjective::getIncrementalCost(const State *s1, const State *s2) const
{
    double c;
    std::pair<double, double> dummy;
    si_->getMotionValidator()->computeMotionCost(s1, s2, c, dummy);
    return c;
}
