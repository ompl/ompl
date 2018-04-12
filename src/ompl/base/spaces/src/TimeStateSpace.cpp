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

#include "ompl/base/spaces/TimeStateSpace.h"
#include "ompl/util/Exception.h"
#include "ompl/tools/config/MagicConstants.h"
#include <limits>

void ompl::base::TimeStateSampler::sampleUniform(State *state)
{
    if (space_->as<TimeStateSpace>()->isBounded())
        state->as<TimeStateSpace::StateType>()->position = rng_.uniformReal(
            space_->as<TimeStateSpace>()->getMinTimeBound(), space_->as<TimeStateSpace>()->getMaxTimeBound());
    else
        state->as<TimeStateSpace::StateType>()->position = 0.0;
}

void ompl::base::TimeStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    state->as<TimeStateSpace::StateType>()->position =
        rng_.uniformReal(near->as<TimeStateSpace::StateType>()->position - distance,
                         near->as<TimeStateSpace::StateType>()->position + distance);
    space_->enforceBounds(state);
}

void ompl::base::TimeStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    state->as<TimeStateSpace::StateType>()->position =
        rng_.gaussian(mean->as<TimeStateSpace::StateType>()->position, stdDev);
    space_->enforceBounds(state);
}

unsigned int ompl::base::TimeStateSpace::getDimension() const
{
    return 1;
}

void ompl::base::TimeStateSpace::setBounds(double minTime, double maxTime)
{
    if (minTime > maxTime)
        throw Exception("The maximum position in time cannot be before the minimum position in time");

    minTime_ = minTime;
    maxTime_ = maxTime;
    bounded_ = true;
}

double ompl::base::TimeStateSpace::getMaximumExtent() const
{
    return bounded_ ? maxTime_ - minTime_ : 1.0;
}

double ompl::base::TimeStateSpace::getMeasure() const
{
    return getMaximumExtent();
}

void ompl::base::TimeStateSpace::enforceBounds(State *state) const
{
    if (bounded_)
    {
        if (state->as<StateType>()->position > maxTime_)
            state->as<StateType>()->position = maxTime_;
        else if (state->as<StateType>()->position < minTime_)
            state->as<StateType>()->position = minTime_;
    }
}

bool ompl::base::TimeStateSpace::satisfiesBounds(const State *state) const
{
    return !bounded_ || (state->as<StateType>()->position >= minTime_ - std::numeric_limits<double>::epsilon() &&
                         state->as<StateType>()->position <= maxTime_ + std::numeric_limits<double>::epsilon());
}

void ompl::base::TimeStateSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->position = source->as<StateType>()->position;
}

unsigned int ompl::base::TimeStateSpace::getSerializationLength() const
{
    return sizeof(double);
}

void ompl::base::TimeStateSpace::serialize(void *serialization, const State *state) const
{
    memcpy(serialization, &state->as<StateType>()->position, sizeof(double));
}

void ompl::base::TimeStateSpace::deserialize(State *state, const void *serialization) const
{
    memcpy(&state->as<StateType>()->position, serialization, sizeof(double));
}

double ompl::base::TimeStateSpace::distance(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->position - state2->as<StateType>()->position);
}

bool ompl::base::TimeStateSpace::equalStates(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->position - state2->as<StateType>()->position) <
           std::numeric_limits<double>::epsilon() * 2.0;
}

void ompl::base::TimeStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    state->as<StateType>()->position =
        from->as<StateType>()->position + (to->as<StateType>()->position - from->as<StateType>()->position) * t;
}

ompl::base::StateSamplerPtr ompl::base::TimeStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<TimeStateSampler>(this);
}

ompl::base::State *ompl::base::TimeStateSpace::allocState() const
{
    return new StateType();
}

void ompl::base::TimeStateSpace::freeState(State *state) const
{
    delete static_cast<StateType *>(state);
}

void ompl::base::TimeStateSpace::registerProjections()
{
    class TimeDefaultProjection : public ProjectionEvaluator
    {
    public:
        TimeDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 1;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(1);
            if (space_->as<TimeStateSpace>()->isBounded())
            {
                bounds_.resize(1);
                bounds_.low[0] = space_->as<TimeStateSpace>()->getMinTimeBound();
                bounds_.high[0] = space_->as<TimeStateSpace>()->getMaxTimeBound();
                cellSizes_[0] = bounds_.getDifference()[0] / magic::PROJECTION_DIMENSION_SPLITS;
            }
            else
                cellSizes_[0] = 1.0;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection(0) = state->as<TimeStateSpace::StateType>()->position;
        }
    };

    registerDefaultProjection(std::make_shared<TimeDefaultProjection>(this));
}

double *ompl::base::TimeStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return index == 0 ? &(state->as<StateType>()->position) : nullptr;
}

void ompl::base::TimeStateSpace::printState(const State *state, std::ostream &out) const
{
    out << "TimeState [";
    if (state != nullptr)
        out << state->as<StateType>()->position;
    else
        out << "nullptr";
    out << ']' << std::endl;
}

void ompl::base::TimeStateSpace::printSettings(std::ostream &out) const
{
    out << "Time state space '" << getName() << "'" << std::endl;
}
