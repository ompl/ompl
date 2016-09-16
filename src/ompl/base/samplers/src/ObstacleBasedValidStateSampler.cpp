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

/* Author: Ioan Sucan */

#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"

ompl::base::ObstacleBasedValidStateSampler::ObstacleBasedValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si), sampler_(si->allocStateSampler())
{
    name_ = "obstacle_based";
}

bool ompl::base::ObstacleBasedValidStateSampler::sample(State *state)
{
    // find invalid state
    unsigned int attempts = 0;
    bool valid = true;
    do
    {
        sampler_->sampleUniform(state);
        valid = si_->isValid(state);
        ++attempts;
    } while (valid && attempts < attempts_);
    if (valid)
        return false;

    // find a valid state
    State *temp = si_->allocState();
    attempts = 0;
    do
    {
        sampler_->sampleUniform(temp);
        valid = si_->isValid(temp);
        ++attempts;
    } while (!valid && attempts < attempts_);

    // keep the last valid state, before collision
    if (valid)
    {
        std::pair<State *, double> fail(state, 0.0);
        si_->checkMotion(temp, state, fail);
    }

    si_->freeState(temp);

    return valid;
}

bool ompl::base::ObstacleBasedValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    // find invalid state nearby
    unsigned int attempts = 0;
    bool valid = true;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->isValid(state);
        ++attempts;
    } while (valid && attempts < attempts_);
    if (valid)
        return false;

    // find a valid state
    State *temp = si_->allocState();
    attempts = 0;
    do
    {
        sampler_->sampleUniform(temp);
        valid = si_->isValid(temp);
        ++attempts;
    } while (!valid && attempts < attempts_);

    // keep the last valid state, before collision
    if (valid)
    {
        std::pair<State *, double> fail(state, 0.0);
        si_->checkMotion(temp, state, fail);
    }

    si_->freeState(temp);

    return valid;
}
