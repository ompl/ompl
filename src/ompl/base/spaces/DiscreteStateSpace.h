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

#ifndef OMPL_BASE_SPACES_DISCRETE_STATE_SPACE_
#define OMPL_BASE_SPACES_DISCRETE_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for discrete states */
        class DiscreteStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            DiscreteStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A space representing discrete states; i.e. there
            are a small number of discrete states the system can be in.
            States are represented as integers [lowerBound, upperBound],
            where lowerBound and upperBound are inclusive.
            States do not wrap around; i.e. the distance between state
            lowerBound and state upperBound is upperBound-lowerBound.
            The dimension of the space is 1. */
        class DiscreteStateSpace : public StateSpace
        {
        public:

            /** \brief The definition of a discrete state */
            class StateType : public State
            {
            public:

                /** \brief The current state - an int in range [lowerBound, upperBound] */
                int value;
            };

            /** \brief Construct a discrete space in wich states can take values in the set [\e lowerBound, \e upperBound] */
            DiscreteStateSpace(int lowerBound, int upperBound) : StateSpace(), lowerBound_(lowerBound), upperBound_(upperBound)
            {
                setName("Discrete" + getName());
                type_ = STATE_SPACE_DISCRETE;
            }

            virtual ~DiscreteStateSpace()
            {
            }

            virtual bool isDiscrete() const;

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();

            /** \brief Returns the number of states possible */
            unsigned int getStateCount() const
            {
                return upperBound_ - lowerBound_ + 1;
            }

            /** \brief Returns the lowest possible state */
            int getLowerBound() const
            {
                return lowerBound_;
            }

            /** \brief Returns the highest possible state */
            int getUpperBound() const
            {
                return upperBound_;
            }

            /** \brief Set the bounds for the states in this space (the states will be in the set [\e lowerBound, \e upperBound] */
            void setBounds(int lowerBound, int upperBound)
            {
                lowerBound_ = lowerBound;
                upperBound_ = upperBound;
            }

            virtual void setup();

        protected:

            /** \brief The lowest integer state */
            int lowerBound_;

            /** \brief The highest integer state */
            int upperBound_;
        };
    }
}

#endif
