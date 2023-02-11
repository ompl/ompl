/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Luis G. Torres */

#include "ompl/base/objectives/StateCostIntegralObjective.h"

ompl::base::StateCostIntegralObjective::StateCostIntegralObjective(const SpaceInformationPtr &si,
                                                                   bool enableMotionCostInterpolation)
  : OptimizationObjective(si), interpolateMotionCost_(enableMotionCostInterpolation)
{
    description_ = "State Cost Integral";
}

ompl::base::Cost ompl::base::StateCostIntegralObjective::stateCost(const State *) const
{
    return Cost(1.0);
}

ompl::base::Cost ompl::base::StateCostIntegralObjective::motionCost(const State *s1, const State *s2) const
{
    if (interpolateMotionCost_)
    {
        Cost totalCost = this->identityCost();

        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);

        State *test1 = si_->cloneState(s1);
        Cost prevStateCost = this->stateCost(test1);
        if (nd > 1)
        {
            State *test2 = si_->allocState();
            for (int j = 1; j < nd; ++j)
            {
                si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                Cost nextStateCost = this->stateCost(test2);
                totalCost = Cost(totalCost.value() +
                                 this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());
                std::swap(test1, test2);
                prevStateCost = nextStateCost;
            }
            si_->freeState(test2);
        }

        // Lastly, add s2
        totalCost = Cost(totalCost.value() +
                         this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

        si_->freeState(test1);

        return totalCost;
    }

    return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
}

ompl::base::Cost ompl::base::StateCostIntegralObjective::motionCostBestEstimate(const State *s1, const State *s2) const
{
    return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
}

bool ompl::base::StateCostIntegralObjective::isMotionCostInterpolationEnabled() const
{
    return interpolateMotionCost_;
}
