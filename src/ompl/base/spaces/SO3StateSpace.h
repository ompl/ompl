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

#ifndef OMPL_BASE_SPACES_SO3_STATE_SPACE_
#define OMPL_BASE_SPACES_SO3_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for SO(3), using quaternion representation  */
        class SO3StateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            SO3StateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            /** \brief To sample unit quaternions uniformly within some given
                distance, we sample a 3-vector from the R^3 tangent space.
                This vector is drawn uniformly random from a 3D ball centered at
                the origin with radius distance. The vector is then "wrapped"
                around S^3 to obtain a unit quaternion uniformly distributed
                around the identity quaternion within given distance. We
                pre-multiply this quaternion with the quaternion near
                to center the distribution around near. */
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            /** \brief Sample a state such that the expected distance between
                mean and state is stdDev.

                To sample a unit quaternion from a Gaussian
                distribution, we sample a 3-vector from the R^3 tangent space
                using a 3D Gaussian with zero mean and covariance matrix equal
                to diag(stdDev^2, stdDev^2, stdDev^2). This vector is "wrapped"
                around S^3 to obtain a Gaussian quaternion with zero mean.
                We pre-multiply this quaternion with the quaternion mean
                to get the desired mean. */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing SO(3). The internal
            representation is done with quaternions. The distance
            between states is the angle between quaternions and
            interpolation is done with slerp. */
        class SO3StateSpace : public StateSpace
        {
        public:


            /** \brief The definition of a state in SO(3) represented as a unit quaternion

                \note The order of the elements matters in this
                definition for the SO3StateUniformSampler::sample()
                function. */
            class StateType : public State
            {
            public:

                /** \brief Set the quaternion from axis-angle representation.  The angle is given in radians. */
                void setAxisAngle(double ax, double ay, double az, double angle);

                /** \brief Set the state to identity -- no rotation */
                void setIdentity();

                /** \brief X component of quaternion vector */
                double x;

                /** \brief Y component of quaternion vector */
                double y;

                /** \brief Z component of quaternion vector */
                double z;

                /** \brief scalar component of quaternion */
                double w;
            };

            SO3StateSpace() : StateSpace()
            {
                setName("SO3" + getName());
                type_ = STATE_SPACE_SO3;
            }

            virtual ~SO3StateSpace()
            {
            }

            /** \brief Compute the norm of a state */
            double norm(const StateType *state) const;

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

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
        };
    }
}

#endif
