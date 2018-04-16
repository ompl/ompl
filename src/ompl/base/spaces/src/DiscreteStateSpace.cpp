/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
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

/* Author: Elizabeth Fudge */

#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/util/Exception.h"
#include <limits>
#include <cstdlib>

void ompl::base::DiscreteStateSampler::sampleUniform(State *state)
{
    state->as<DiscreteStateSpace::StateType>()->value = rng_.uniformInt(
        space_->as<DiscreteStateSpace>()->getLowerBound(), space_->as<DiscreteStateSpace>()->getUpperBound());
}

void ompl::base::DiscreteStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    const auto d = (int)floor(distance + 0.5);
    state->as<DiscreteStateSpace::StateType>()->value = rng_.uniformInt(
        near->as<DiscreteStateSpace::StateType>()->value - d, near->as<DiscreteStateSpace::StateType>()->value + d);
    space_->enforceBounds(state);
}

void ompl::base::DiscreteStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    state->as<DiscreteStateSpace::StateType>()->value =
        (int)floor(rng_.gaussian(mean->as<DiscreteStateSpace::StateType>()->value, stdDev) + 0.5);
    space_->enforceBounds(state);
}

bool ompl::base::DiscreteStateSpace::isDiscrete() const
{
    return true;
}

unsigned int ompl::base::DiscreteStateSpace::getDimension() const
{
    return 1;
}

double ompl::base::DiscreteStateSpace::getMaximumExtent() const
{
    return upperBound_ - lowerBound_;
}

double ompl::base::DiscreteStateSpace::getMeasure() const
{
    return upperBound_ - lowerBound_ + 1.0;
}

void ompl::base::DiscreteStateSpace::enforceBounds(State *state) const
{
    if (state->as<StateType>()->value < lowerBound_)
        state->as<StateType>()->value = lowerBound_;
    else if (state->as<StateType>()->value > upperBound_)
        state->as<StateType>()->value = upperBound_;
}

bool ompl::base::DiscreteStateSpace::satisfiesBounds(const State *state) const
{
    return state->as<StateType>()->value >= lowerBound_ && state->as<StateType>()->value <= upperBound_;
}

void ompl::base::DiscreteStateSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->value = source->as<StateType>()->value;
}

unsigned int ompl::base::DiscreteStateSpace::getSerializationLength() const
{
    return sizeof(int);
}

void ompl::base::DiscreteStateSpace::serialize(void *serialization, const State *state) const
{
    memcpy(serialization, &state->as<StateType>()->value, sizeof(int));
}

void ompl::base::DiscreteStateSpace::deserialize(State *state, const void *serialization) const
{
    memcpy(&state->as<StateType>()->value, serialization, sizeof(int));
}

double ompl::base::DiscreteStateSpace::distance(const State *state1, const State *state2) const
{
    return abs(state1->as<StateType>()->value - state2->as<StateType>()->value);
}

bool ompl::base::DiscreteStateSpace::equalStates(const State *state1, const State *state2) const
{
    return state1->as<StateType>()->value == state2->as<StateType>()->value;
}

void ompl::base::DiscreteStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    state->as<StateType>()->value = (int)floor(from->as<StateType>()->value +
                                               (to->as<StateType>()->value - from->as<StateType>()->value) * t + 0.5);
}

ompl::base::StateSamplerPtr ompl::base::DiscreteStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<DiscreteStateSampler>(this);
}

ompl::base::State *ompl::base::DiscreteStateSpace::allocState() const
{
    return new StateType();
}

void ompl::base::DiscreteStateSpace::freeState(State *state) const
{
    delete static_cast<StateType *>(state);
}

void ompl::base::DiscreteStateSpace::registerProjections()
{
    class DiscreteDefaultProjection : public ProjectionEvaluator
    {
    public:
        DiscreteDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 1;
        }

        void defaultCellSizes() override
        {
            bounds_.resize(1);
            bounds_.low[0] = space_->as<DiscreteStateSpace>()->lowerBound_;
            bounds_.high[0] = space_->as<DiscreteStateSpace>()->upperBound_;
            cellSizes_.resize(1);
            cellSizes_[0] = 1.0;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection(0) = state->as<DiscreteStateSpace::StateType>()->value;
        }
    };

    registerDefaultProjection(std::make_shared<DiscreteDefaultProjection>(this));
}

void ompl::base::DiscreteStateSpace::setup()
{
    if (lowerBound_ > upperBound_)
        throw Exception("Lower bound cannot be larger than upper bound for a discrete space");
    StateSpace::setup();
}

void ompl::base::DiscreteStateSpace::printState(const State *state, std::ostream &out) const
{
    out << "DiscreteState [";
    if (state != nullptr)
        out << state->as<StateType>()->value;
    else
        out << "nullptr";
    out << ']' << std::endl;
}

void ompl::base::DiscreteStateSpace::printSettings(std::ostream &out) const
{
    out << "Discrete state space '" << getName() << "' with bounds [" << lowerBound_ << ", " << upperBound_ << "]"
        << std::endl;
}
