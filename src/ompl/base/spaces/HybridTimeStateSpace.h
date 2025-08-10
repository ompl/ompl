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

#ifndef OMPL_BASE_SPACES_HYBRID_TIME_STATE_SPACE_
#define OMPL_BASE_SPACES_HYBRID_TIME_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief State space sampler for time */
        class HybridTimeStateSampler : public StateSampler
        {
        public:
            /** \brief Constructor */
            HybridTimeStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
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
        class HybridTimeStateSpace : public StateSpace
        {
        public:
            /** \brief The definition of a time state */
            class StateType : public State
            {
            public:
                /** \brief The position in time */
                double position;

                /** \brief The number of jumps */
                unsigned int jumps;
            };

            HybridTimeStateSpace()
            {
                setName("Hybrid Time" + getName());
                type_ = STATE_SPACE_TIME;   // TODO: add STATE_SPACE_HYBRID_TIME to ompl::base::StateSpaceType
            }

            ~HybridTimeStateSpace() override = default;

            unsigned int getDimension() const override;

            double getMaximumExtent() const override;

            double getMeasure() const override;

            /** \brief Set the minimum and maximum time bounds. This
                will make the state space switch into bounded time
                mode. If this function is not called, sampling time
                will always produce position = 0, enforceBounds() is a no-op,
                satisfiesBounds() always returns true and
                getMaximumExtent() returns 1. */
            void setTimeBounds(double minTime, double maxTime);

            /** \brief Set the minimum and maximum jump bounds. This
                will make the state space switch into bounded time
                mode. If this function is not called, sampling jump
                will always produce jumps = 0, enforceBounds() is a no-op,
                satisfiesBounds() always returns true and
                getMaximumExtent() returns 1. */
            void setJumpBounds(unsigned int minJumps, unsigned int maxJumps);

            /** \brief Get the minimum allowed value of \e position in a state. The function returns 0 if time is not
             * bounded. */
            double getMinTimeBound() const
            {
                return minTime_;
            }

            /** \brief Get the maximum allowed value of \e position in a state. The function returns 0 if time is not
             * bounded. */
            double getMaxTimeBound() const
            {
                return maxTime_;
            }

            /** \brief Get the minimum allowed value of \e jumps in a state. The function returns 0 if jumps is not
             * bounded. */
            double getMinJumpsBound() const
            {
                return minJumps_;
            }

            /** \brief Get the maximum allowed value of \e jumps in a state. The function returns 0 if jumps is not
             * bounded. */
            double getMaxJumpBound() const
            {
                return maxJumps_;
            }

            /** \brief Check if the time is bounded or not */
            bool isTimeBounded() const
            {
                return timeBounded_;
            }

            /** \brief Check if the time is bounded or not */
            bool areJumpsBounded() const
            {
                return jumpsBounded_;
            }

            void enforceBounds(State *state) const override;

            bool satisfiesBounds(const State *state) const override;

            void copyState(State *destination, const State *source) const override;

            unsigned int getSerializationLength() const override;

            void serialize(void *serialization, const State *state) const override;

            void deserialize(State *state, const void *serialization) const override;

            double distance(const State *state1, const State *state2) const override;

            bool equalStates(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;

            StateSamplerPtr allocDefaultStateSampler() const override;

            State *allocState() const override;

            void freeState(State *state) const override;

            double *getValueAddressAtIndex(State *state, unsigned int index) const override;

            void printState(const State *state, std::ostream &out) const override;

            void printSettings(std::ostream &out) const override;

            // void registerProjections() override;

        protected:
            /** \brief Flag indicating whether the state space is considering bounds or not */
            bool timeBounded_{false};

            /** \brief The minimum point in time considered by the state space (if bounds are used) */
            double minTime_{0.0};

            /** \brief The maximum point in time considered by the state space (if bounds are used) */
            double maxTime_{0.0};

            /** \brief Flag indicating whether the state space is considering bounds or not */
            bool jumpsBounded_{false};

            /** \brief The minimum point in time considered by the state space (if bounds are used) */
            unsigned int minJumps_{0};

            /** \brief The maximum point in time considered by the state space (if bounds are used) */
            unsigned int maxJumps_{0};
        };
    }
}

#endif