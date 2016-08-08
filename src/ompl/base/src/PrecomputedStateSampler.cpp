/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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

#include "ompl/base/PrecomputedStateSampler.h"
#include "ompl/base/StateSpace.h"
#include "ompl/util/Exception.h"

ompl::base::PrecomputedStateSampler::PrecomputedStateSampler(const StateSpace *space,
                                                             const std::vector<const State *> &states)
  : StateSampler(space), states_(states)
{
    if (states_.empty())
        throw Exception("Empty set of states to sample from was specified");
    minStateIndex_ = 0;
    maxStateIndex_ = states_.size() - 1;
}

ompl::base::PrecomputedStateSampler::PrecomputedStateSampler(const StateSpace *space,
                                                             const std::vector<const State *> &states,
                                                             std::size_t minStateIndex, std::size_t maxStateIndex)
  : StateSampler(space), states_(states), minStateIndex_(minStateIndex), maxStateIndex_(maxStateIndex)
{
    if (states_.empty())
        throw Exception("Empty set of states to sample from was specified");
    if (minStateIndex > maxStateIndex)
        throw Exception("Minimum state index cannot be larger than maximum state index");
    if (maxStateIndex >= states_.size())
        throw Exception("Index range out of bounds");
}

void ompl::base::PrecomputedStateSampler::sampleUniform(State *state)
{
    space_->copyState(state, states_[rng_.uniformInt(minStateIndex_, maxStateIndex_)]);
}

void ompl::base::PrecomputedStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    int index = rng_.uniformInt(minStateIndex_, maxStateIndex_);
    double dist = space_->distance(near, states_[index]);
    if (dist > distance)
        space_->interpolate(near, states_[index], distance / dist, state);
    else
        space_->copyState(state, states_[index]);
}

void ompl::base::PrecomputedStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
}
