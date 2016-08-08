/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Javier V. Gómez*/

#include "ompl/geometric/planners/cforest/CForestStateSampler.h"

void ompl::base::CForestStateSampler::sampleUniform(State *state)
{
    if (!statesToSample_.empty())
        getNextSample(state);
    else
        sampler_->sampleUniform(state);
}

void ompl::base::CForestStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    if (!statesToSample_.empty())
        getNextSample(state);
    else
        sampler_->sampleUniformNear(state, near, distance);
}

void ompl::base::CForestStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    if (!statesToSample_.empty())
        getNextSample(state);
    else
        sampler_->sampleGaussian(state, mean, stdDev);
}

void ompl::base::CForestStateSampler::setStatesToSample(const std::vector<const State *> &states)
{
    std::lock_guard<std::mutex> slock(statesLock_);
    for (auto &i : statesToSample_)
        space_->freeState(i);
    statesToSample_.clear();

    statesToSample_.reserve(states.size());
    // push in reverse order, so that the states are popped in order in getNextSample()
    for (auto state : states)
    {
        State *s = space_->allocState();
        space_->copyState(s, state);
        statesToSample_.push_back(s);
    }
}

void ompl::base::CForestStateSampler::getNextSample(State *state)
{
    std::lock_guard<std::mutex> slock(statesLock_);
    space_->copyState(state, statesToSample_.back());
    space_->freeState(statesToSample_.back());
    statesToSample_.pop_back();
}

void ompl::base::CForestStateSampler::clear()
{
    std::lock_guard<std::mutex> slock(statesLock_);
    for (auto &i : statesToSample_)
        space_->freeState(i);
    statesToSample_.clear();
    sampler_.reset();
}
