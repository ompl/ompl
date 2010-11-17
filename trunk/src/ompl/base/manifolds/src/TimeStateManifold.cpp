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

#include "ompl/base/manifolds/TimeStateManifold.h"
#include "ompl/util/Exception.h"
#include <limits>

void ompl::base::TimeStateSampler::sampleUniform(State *state)
{
    if (manifold_->as<TimeStateManifold>()->isBounded())
        state->as<TimeStateManifold::StateType>()->position = rng_.uniformReal(manifold_->as<TimeStateManifold>()->getMinTimeBound(),
                                                                               manifold_->as<TimeStateManifold>()->getMaxTimeBound());
    else
        state->as<TimeStateManifold::StateType>()->position = 0.0;
}

void ompl::base::TimeStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    state->as<TimeStateManifold::StateType>()->position =
        rng_.uniformReal(near->as<TimeStateManifold::StateType>()->position - distance,
                         near->as<TimeStateManifold::StateType>()->position + distance);
    manifold_->enforceBounds(state);
}

void ompl::base::TimeStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    state->as<TimeStateManifold::StateType>()->position = 
        rng_.gaussian(mean->as<TimeStateManifold::StateType>()->position, stdDev);
    manifold_->enforceBounds(state);
}

unsigned int ompl::base::TimeStateManifold::getDimension(void) const
{
    return 1;
}

void ompl::base::TimeStateManifold::setBounds(double minTime, double maxTime)
{
    if (minTime > maxTime)
        throw Exception("The maximum position in time cannot be before the minimum position in time");
    
    minTime_ = minTime;
    maxTime_ = maxTime;
    bounded_ = true;
}

double ompl::base::TimeStateManifold::getMaximumExtent(void) const
{
    return bounded_ ? maxTime_ - minTime_ : 1.0;
}

void ompl::base::TimeStateManifold::enforceBounds(State *state) const
{
    if (bounded_)
    {
        if (state->as<StateType>()->position > maxTime_)
            state->as<StateType>()->position = maxTime_;
        else
            if (state->as<StateType>()->position < minTime_)
                state->as<StateType>()->position = minTime_;
    }
}    

bool ompl::base::TimeStateManifold::satisfiesBounds(const State *state) const
{
    return !bounded_ || (state->as<StateType>()->position >= minTime_ - std::numeric_limits<double>::epsilon() && 
                         state->as<StateType>()->position <= maxTime_ + std::numeric_limits<double>::epsilon());
}

void ompl::base::TimeStateManifold::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->position = source->as<StateType>()->position;
}

double ompl::base::TimeStateManifold::distance(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->position - state2->as<StateType>()->position);
}

bool ompl::base::TimeStateManifold::equalStates(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->position - state2->as<StateType>()->position) < std::numeric_limits<double>::epsilon() * 2.0;
}

void ompl::base::TimeStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    state->as<StateType>()->position = from->as<StateType>()->position +
        (to->as<StateType>()->position - from->as<StateType>()->position) * t;
}

ompl::base::ManifoldStateSamplerPtr ompl::base::TimeStateManifold::allocStateSampler(void) const 
{
    return ManifoldStateSamplerPtr(new TimeStateSampler(this));
}

ompl::base::State* ompl::base::TimeStateManifold::allocState(void) const
{
    return new StateType();
}

void ompl::base::TimeStateManifold::freeState(State *state) const
{
    delete static_cast<StateType*>(state);
}

void ompl::base::TimeStateManifold::registerProjections(void)
{
    class TimeDefaultProjection : public ProjectionEvaluator
    {
    public:
        
        TimeDefaultProjection(const StateManifold *manifold) : ProjectionEvaluator(manifold)
        {
            std::vector<double> dims(1);
            if (manifold->as<TimeStateManifold>()->isBounded())
                dims[0] = (manifold->as<TimeStateManifold>()->getMaxTimeBound() - manifold->as<TimeStateManifold>()->getMinTimeBound()) / 10.0;
            else
                dims[0] = 1.0;
            setCellDimensions(dims);
        }
        
        virtual unsigned int getDimension(void) const
        {
            return 1;
        }
        
        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            projection.values[0] = state->as<TimeStateManifold::StateType>()->position;
        }
    };
    
    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new TimeDefaultProjection(this))));
}

void ompl::base::TimeStateManifold::printState(const State *state, std::ostream &out) const
{
    out << "TimeState [";
    if (state)
        out << state->as<StateType>()->position;
    else
        out << "NULL";
    out << ']' << std::endl;
}

void ompl::base::TimeStateManifold::printSettings(std::ostream &out) const
{
    out << "Time state manifold '" << name_ << "'" << std::endl;
}
