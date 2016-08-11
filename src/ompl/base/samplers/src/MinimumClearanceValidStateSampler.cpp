/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Find a valid sample with a minimum distance to nearby obstacles
           (clearance threshold)
*/

#include "ompl/base/samplers/MinimumClearanceValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"

ompl::base::MinimumClearanceValidStateSampler::MinimumClearanceValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si), sampler_(si->allocStateSampler()), clearance_(1)
{
    name_ = "min_clearance";
    params_.declareParam<double>("min_obstacle_clearance",
                                 [this](double c)
                                 {
                                     setMinimumObstacleClearance(c);
                                 },
                                 [this]
                                 {
                                     return getMinimumObstacleClearance();
                                 });
}

bool ompl::base::MinimumClearanceValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;
    double dist = 0.0;
    do
    {
        sampler_->sampleUniform(state);
        valid = si_->getStateValidityChecker()->isValid(state, dist);

        // Also check for distance to nearest obstacle and invalidate if too close
        if (dist < clearance_)
        {
            valid = false;
        }

        ++attempts;
    } while (!valid && attempts < attempts_);

    return valid;
}

bool ompl::base::MinimumClearanceValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    double dist = 0.0;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->getStateValidityChecker()->isValid(state, dist);

        // Also check for distance to nearest obstacle and invalidate if too close
        if (dist < clearance_)
        {
            valid = false;
        }

        ++attempts;
    } while (!valid && attempts < attempts_);

    return valid;
}
