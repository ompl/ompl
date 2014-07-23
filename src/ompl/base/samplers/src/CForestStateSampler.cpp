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

/* Author: Javier V. Gómez*/

#include "ompl/base/samplers/CForestStateSampler.h"


void ompl::base::CForestStateSampler::sampleUniform(State *state)
{
    if (statesToSample_.size() > 0)
        getNextSample(state);
    else
        sampler_->sampleUniform(state);
}

void ompl::base::CForestStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    if (statesToSample_.size() > 0)
        getNextSample(state);
    else
        sampler_->sampleUniformNear(state, near, distance);
}

void ompl::base::CForestStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    if (statesToSample_.size() > 0)
        getNextSample(state);
    else
        sampler_->sampleGaussian(state, mean, stdDev);
}

void ompl::base::CForestStateSampler::addStateToSample(const State *state)
{
    State *copy = space_->allocState();
    space_->copyState(copy, state);
    statesToSample_.push_back(copy);
}

void ompl::base::CForestStateSampler::setStatesToSample(const std::vector<State *> &states)
{
    statesToSample_.clear();
    statesToSample_.reserve(states.size());
    for (size_t i = 0; i < states.size(); ++i)
        addStateToSample(states[i]);
}

void ompl::base::CForestStateSampler::getNextSample(State *state)
{
    space_->copyState(state, statesToSample_.back());
    space_->freeState(statesToSample_.back());
    statesToSample_.pop_back();
}









/*

ompl::base::CForestStateSampler::CForestStateSampler(const StateSpace *space) : StateSampler(space)
{
    name_ = "cforest";
}

bool ompl::base::CForestStateSampler::sample(State *state)
{
    bool valid = false;

    if (statesToSample_.size() > 0)
    {
        getNextSample(state);
        valid = true;
    }
    else
    {
        unsigned int attempts = 0;
        do
        {
            sampler_->sampleUniform(state);
            valid = si_->isValid(state);
            ++attempts;
        } while (!valid && attempts < attempts_);
    }
     return valid;
}

bool ompl::base::CForestStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->isValid(state);
        ++attempts;
    } while (!valid && attempts < attempts_);
    return valid;
}
*/
