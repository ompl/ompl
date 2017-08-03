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

/* Authors: Caleb Voss */

#ifndef OMPL_EXTENSION_MORSE_STATE_SPACE_
#define OMPL_EXTENSION_MORSE_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/extensions/morse/MorseEnvironment.h"

namespace ompl
{
    namespace base
    {
        /** \brief State space representing MORSE states */
        class MorseStateSpace : public CompoundStateSpace
        {
        public:
            /** \brief MORSE State. This is a compound state that allows accessing the properties of the bodies the
             * state space is constructed for. */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() : CompoundStateSpace::StateType()
                {
                }
            };

            /** \brief Construct a state space representing MORSE states.

                This will be a compound state space with 4 components for
                each body in \e env.stateBodies_. The 4 subspaces
                constructed for each body are: position
                (R<sup>3</sup>), linear velocity (R<sup>3</sup>),
                angular velocity (R<sup>3</sup>) and orientation
                (SO(3)). Default bounds are set by calling setDefaultBounds().

                \param env the environment to construct the state space for
                \param positionWeight the weight to pass to CompoundStateSpace::addSubspace() for position subspaces
                \param linVelWeight the weight to pass to CompoundStateSpace::addSubspace() for linear velocity
               subspaces
                \param angVelWeight the weight to pass to CompoundStateSpace::addSubspace() for angular velocity
               subspaces
                \param orientationWeight the weight to pass to CompoundStateSpace::addSubspace() for orientation
               subspaces
            */
            MorseStateSpace(const MorseEnvironmentPtr &env, double positionWeight = 1.0, double linVelWeight = 0.5,
                            double angVelWeight = 0.5, double orientationWeight = 1.0);

            virtual ~MorseStateSpace()
            {
            }

            /** \brief Get the MORSE environment this state space corresponds to */
            const MorseEnvironmentPtr &getEnvironment() const
            {
                return env_;
            }

            /** \brief Get the number of bodies this state space represents */
            unsigned int getNrBodies() const
            {
                return env_->rigidBodies_;
            }

            /** \brief Set the bounds given by the MorseEnvironment */
            void setBounds();

            /** \brief Set the bounds for each of the position subspaces */
            void setPositionBounds(const RealVectorBounds &bounds);

            /** \brief Set the bounds for each of the linear velocity subspaces */
            void setLinearVelocityBounds(const RealVectorBounds &bounds);

            /** \brief Set the bounds for each of the angular velocity subspaces */
            void setAngularVelocityBounds(const RealVectorBounds &bounds);

            /** \brief Read the parameters of the MORSE bodies and store
                them in \e state. */
            void readState(State *state) const;

            /** \brief Set the parameters of the MORSE bodies to be the
                ones read from \e state. */
            void writeState(const State *state) const;

            /** \brief This function checks whether a state satisfies its bounds */
            bool satisfiesBounds(const State *state) const override;

            State *allocState() const override;
            void freeState(State *state) const override;
            void copyState(State *destination, const State *source) const override;
            void interpolate(const State *from, const State *to, double t, State *state) const override;

            StateSamplerPtr allocDefaultStateSampler() const override;
            StateSamplerPtr allocStateSampler() const override;

        protected:
            /** \brief Representation of the MORSE parameters OMPL needs to plan */
            MorseEnvironmentPtr env_;
        };
    }
}

#endif
