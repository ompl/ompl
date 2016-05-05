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

#include "ompl/control/ControlSampler.h"
#include "ompl/control/ControlSpace.h"

void ompl::control::ControlSampler::sample(Control *control, const base::State * /* state */)
{
    sample(control);
}

void ompl::control::ControlSampler::sampleNext(Control *control, const Control * /* previous */)
{
    sample(control);
}

void ompl::control::ControlSampler::sampleNext(Control *control, const Control * /* previous */,
                                               const base::State * /* state */)
{
    sample(control);
}

unsigned int ompl::control::ControlSampler::sampleStepCount(unsigned int minSteps, unsigned int maxSteps)
{
    return rng_.uniformInt(minSteps, maxSteps);
}

void ompl::control::CompoundControlSampler::addSampler(const ControlSamplerPtr &sampler)
{
    samplers_.push_back(sampler);
    samplerCount_ = samplers_.size();
}

void ompl::control::CompoundControlSampler::sample(Control *control)
{
    Control **comps = static_cast<CompoundControl *>(control)->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        samplers_[i]->sample(comps[i]);
}

void ompl::control::CompoundControlSampler::sample(Control *control, const base::State *state)
{
    Control **comps = static_cast<CompoundControl *>(control)->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        samplers_[i]->sample(comps[i], state);
}

void ompl::control::CompoundControlSampler::sampleNext(Control *control, const Control *previous)
{
    Control **comps = static_cast<CompoundControl *>(control)->components;
    const Control *const *prev = static_cast<const CompoundControl *>(previous)->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        samplers_[i]->sampleNext(comps[i], prev[i]);
}

void ompl::control::CompoundControlSampler::sampleNext(Control *control, const Control *previous,
                                                       const base::State *state)
{
    Control **comps = static_cast<CompoundControl *>(control)->components;
    const Control *const *prev = static_cast<const CompoundControl *>(previous)->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        samplers_[i]->sampleNext(comps[i], prev[i], state);
}
