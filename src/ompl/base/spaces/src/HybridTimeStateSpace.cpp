/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of Santa Cruz Hybrid Systems Laboratory
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
 *   * Neither the name of the University of Santa Cruz nor the names of 
 *     its contributors may be used to endorse or promote products derived
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

/* Author: Beverly Xu */

#include "ompl/base/spaces/HybridTimeStateSpace.h"
#include "ompl/util/Exception.h"
#include <limits>

void ompl::base::HybridTimeStateSampler::sampleUniform(State *state)
{
    if (space_->as<HybridTimeStateSpace>()->isTimeBounded()) {
        state->as<HybridTimeStateSpace::StateType>()->position = rng_.uniformReal(
            space_->as<HybridTimeStateSpace>()->getMinTimeBound(), space_->as<HybridTimeStateSpace>()->getMaxTimeBound());
        state->as<HybridTimeStateSpace::StateType>()->jumps = rng_.uniformInt(
            space_->as<HybridTimeStateSpace>()->getMinJumpsBound(), space_->as<HybridTimeStateSpace>()->getMaxJumpBound());

    }
    else
        state->as<HybridTimeStateSpace::StateType>()->position = 0.0;
}

void ompl::base::HybridTimeStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    state->as<HybridTimeStateSpace::StateType>()->position =
        rng_.uniformReal(near->as<HybridTimeStateSpace::StateType>()->position - distance,
                         near->as<HybridTimeStateSpace::StateType>()->position + distance);
    space_->enforceBounds(state);
}

void ompl::base::HybridTimeStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    state->as<HybridTimeStateSpace::StateType>()->position =
        rng_.gaussian(mean->as<HybridTimeStateSpace::StateType>()->position, stdDev);
    space_->enforceBounds(state);
}

void ompl::base::HybridTimeStateSpace::interpolate(const State *from, const State *to, double t, State *state) const {
    (void)from, (void)to, (void)t, (void)state;
}

unsigned int ompl::base::HybridTimeStateSpace::getDimension() const
{
    return 2;
}

void ompl::base::HybridTimeStateSpace::setTimeBounds(double minTime, double maxTime)
{
    if (minTime > maxTime)
        throw Exception("The maximum position in time cannot be before the minimum position in time");

    minTime_ = minTime;
    maxTime_ = maxTime;
    timeBounded_ = true;
}

void ompl::base::HybridTimeStateSpace::setJumpBounds(unsigned int minJumps, unsigned int maxJumps)
{
    if (minJumps > maxJumps)
        throw Exception("The maximum jumps cannot be before the minimum jumps");

    minJumps_ = minJumps;
    maxJumps_ = maxJumps;
    jumpsBounded_ = true;
}

double ompl::base::HybridTimeStateSpace::getMaximumExtent() const
{
    return timeBounded_ ? maxTime_ - minTime_ : 1.0;
}

double ompl::base::HybridTimeStateSpace::getMeasure() const
{
    return getMaximumExtent();
}

void ompl::base::HybridTimeStateSpace::enforceBounds(State *state) const
{
    if (timeBounded_)
    {
        if (state->as<StateType>()->position > maxTime_)
            state->as<StateType>()->position = maxTime_;
        else if (state->as<StateType>()->position < minTime_)
            state->as<StateType>()->position = minTime_;
    }
    if (jumpsBounded_)
    {
        if (state->as<StateType>()->jumps > maxJumps_)
            state->as<StateType>()->jumps = maxJumps_;
        else if (state->as<StateType>()->jumps < minJumps_)
            state->as<StateType>()->jumps = minJumps_;
    }
}

bool ompl::base::HybridTimeStateSpace::satisfiesBounds(const State *state) const
{
    return (!timeBounded_ || (state->as<StateType>()->position >= minTime_ - std::numeric_limits<double>::epsilon() &&
                         state->as<StateType>()->position <= maxTime_ + std::numeric_limits<double>::epsilon())) &&
                         (!jumpsBounded_ || (state->as<StateType>()->jumps >= minJumps_ - std::numeric_limits<double>::epsilon() &&
                         state->as<StateType>()->jumps <= maxJumps_ + std::numeric_limits<double>::epsilon()));
}

void ompl::base::HybridTimeStateSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->position = source->as<StateType>()->position;
    destination->as<StateType>()->jumps = source->as<StateType>()->jumps;
}

unsigned int ompl::base::HybridTimeStateSpace::getSerializationLength() const
{
    return sizeof(double);
}

void ompl::base::HybridTimeStateSpace::serialize(void *serialization, const State *state) const
{
    memcpy(serialization, &state->as<StateType>()->position, sizeof(double));
}

void ompl::base::HybridTimeStateSpace::deserialize(State *state, const void *serialization) const
{
    memcpy(&state->as<StateType>()->position, serialization, sizeof(double));
}

double ompl::base::HybridTimeStateSpace::distance(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->position - state2->as<StateType>()->position);
}

bool ompl::base::HybridTimeStateSpace::equalStates(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->position - state2->as<StateType>()->position) <
           std::numeric_limits<double>::epsilon() * 2.0;
}

ompl::base::StateSamplerPtr ompl::base::HybridTimeStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<HybridTimeStateSampler>(this);
}

ompl::base::State *ompl::base::HybridTimeStateSpace::allocState() const
{
    return new StateType();
}

void ompl::base::HybridTimeStateSpace::freeState(State *state) const
{
    delete static_cast<StateType *>(state);
}

double *ompl::base::HybridTimeStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    if (index == 0)
        return &(state->as<StateType>()->position);
    else if (index == 1) { 
        return std::make_shared<double>(state->as<StateType>()->jumps).get();
    }
    return nullptr;
}

void ompl::base::HybridTimeStateSpace::printState(const State *state, std::ostream &out) const
{
    out << "TimeState [";
    if (state != nullptr)
        out << state->as<StateType>()->position << ", " << state->as<StateType>()->jumps;
    else
        out << "nullptr";
    out << ']' << std::endl;
}

void ompl::base::HybridTimeStateSpace::printSettings(std::ostream &out) const
{
    out << "Time state space '" << getName() << "'" << std::endl;
}