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

#ifndef OMPL_BASE_SPACES_SO2_STATE_SPACE_
#define OMPL_BASE_SPACES_SO2_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief State space sampler for SO(2) */
        class SO2StateSampler : public StateSampler
        {
        public:
            /** \brief Constructor */
            SO2StateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

        /** \brief A state space representing SO(2). The distance
            function and interpolation take into account angle
            wrapping. */
        class SO2StateSpace : public StateSpace
        {
        public:
            /** \brief The definition of a state in SO(2) */
            class StateType : public State
            {
            public:
                /** \brief Set the state to identity -- no rotation (value = 0.0) */
                void setIdentity()
                {
                    value = 0.0;
                }

                /** \brief The value of the angle in the interval (-Pi, Pi] */
                double value;
            };

            SO2StateSpace()
            {
                setName("SO2" + getName());
                type_ = STATE_SPACE_SO2;
            }

            ~SO2StateSpace() override = default;

            unsigned int getDimension() const override;

            double getMaximumExtent() const override;

            double getMeasure() const override;

            /** \brief Normalize the value of the state to the interval [-Pi, Pi) */
            void enforceBounds(State *state) const override;

            /** \brief Check if the value of the state is in the interval [-Pi, Pi) */
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

            void registerProjections() override;
        };
    }
}

#endif
