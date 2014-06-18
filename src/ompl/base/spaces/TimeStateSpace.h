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

#ifndef OMPL_BASE_SPACES_TIME_STATE_SPACE_
#define OMPL_BASE_SPACES_TIME_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for time */
        class TimeStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            TimeStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing time. The time can be
            unbounded, in which case enforceBounds() is a no-op,
            satisfiesBounds() always returns true, sampling uniform
            time states always produces time 0 and getMaximumExtent()
            returns 1. If time is bounded (setBounds() has been
            previously called), the state space behaves as
            expected. After construction, the state space is
            unbounded. isBounded() can be used to check if the state space
            is bounded or not. */
        class TimeStateSpace : public StateSpace
        {
        public:

            /** \brief The definition of a time state */
            class StateType : public State
            {
            public:

                /** \brief The position in time */
                double position;
            };

            TimeStateSpace() : StateSpace(), bounded_(false), minTime_(0.0), maxTime_(0.0)
            {
                setName("Time" + getName());
                type_ = STATE_SPACE_TIME;
            }

            virtual ~TimeStateSpace()
            {
            }

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            /** \brief Set the minimum and maximum time bounds. This
                will make the state space switch into bounded time
                mode. If this function is not called, sampling time
                will always produce position = 0, enforceBounds() is a no-op,
                satisfiesBounds() always returns true and
                getMaximumExtent() returns 1. */
            void setBounds(double minTime, double maxTime);

            /** \brief Get the minimum allowed value of \e position in a state. The function returns 0 if time is not bounded. */
            double getMinTimeBound() const
            {
                return minTime_;
            }

            /** \brief Get the maximum allowed value of \e position in a state. The function returns 0 if time is not bounded. */
            double getMaxTimeBound() const
            {
                return maxTime_;
            }

            /** \brief Check if the time is bounded or not */
            bool isBounded() const
            {
                return bounded_;
            }

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();

        protected:

            /** \brief Flag indicating whether the state space is considering bounds or not */
            bool   bounded_;

            /** \brief The minimum point in time considered by the state space (if bounds are used) */
            double minTime_;

            /** \brief The maximum point in time considered by the state space (if bounds are used) */
            double maxTime_;

        };
    }
}

#endif
