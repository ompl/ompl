/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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

/* Author: Clayton W. Ramsey */

#include "ompl/base/objectives/FlatEffortObjective.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

namespace ompl::base
{
    FlatEffortObjective::FlatEffortObjective(const SpaceInformationPtr &si) : OptimizationObjective(si)
    {
        description_ = "Flat Effort";

        stateSpace_ = dynamic_cast<const FlatStateSpace *>(si->getStateSpace().get());
        if (stateSpace_ == nullptr)
            throw Exception("FlatEffortObjective needs a FlatStateSpace");
    }

    Cost FlatEffortObjective::stateCost(const State *) const
    {
        return identityCost();
    }

    Cost FlatEffortObjective::motionCost(const State *s1, const State *s2) const
    {
        return Cost(stateSpace_->steeringCost(s1, s2));
    }

    Cost FlatEffortObjective::motionCostHeuristic(const State *s1, const State *s2) const
    {
        // The steering polynomial ignores obstacles, so whatever a planner ends up doing to get from s1 to
        // s2 costs at least this much.
        return motionCost(s1, s2);
    }

    Cost FlatEffortObjective::motionCostBestEstimate(const State *s1, const State *s2) const
    {
        return motionCost(s1, s2);
    }
}  // namespace ompl::base
