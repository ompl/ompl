/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#include "ompl/base/objectives/MinimizeArrivalTime.h"

ompl::base::MinimizeArrivalTime::MinimizeArrivalTime(const ompl::base::SpaceInformationPtr &si)
  : OptimizationObjective(si) {}

ompl::base::Cost ompl::base::MinimizeArrivalTime::stateCost(const ompl::base::State *s) const
{
    return base::Cost(s->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position);
}

ompl::base::Cost ompl::base::MinimizeArrivalTime::motionCost(const ompl::base::State *s1,
                                                             const ompl::base::State *s2) const
{
    return combineCosts(stateCost(s1), stateCost(s2));
}

ompl::base::Cost ompl::base::MinimizeArrivalTime::combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const
{
    return c1.value() > c2.value() ? c1 : c2;
}

ompl::base::Cost ompl::base::MinimizeArrivalTime::identityCost() const
{
    return base::Cost(-std::numeric_limits<double>::infinity());
}

