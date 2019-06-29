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

#ifndef OMPL_EXTENSION_OPENDE_STATE_SPACE_
#define OMPL_EXTENSION_OPENDE_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/extensions/ode/OpenDEEnvironment.h"

namespace ompl
{
    namespace control
    {
        /** \brief State space representing OpenDE states */
        class OpenDEStateSpace : public base::CompoundStateSpace
        {
        public:
            enum
            {
                /** \brief Index of bit in StateType::collision indicating whether it is known if a state is in
                   collision or not. Initially this is 0. The value of this bit is updated by
                   OpenDEStateSpace::evaluateCollision() and OpenDEControlSpace::propagate(). */
                STATE_COLLISION_KNOWN_BIT = 0,
                /** \brief Index of bit in StateType::collision indicating whether a state is in collision or not.
                   Initially the value of this field is unspecified. The value gains meaning (1 or 0) when
                   OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT becomes 1. The value of this bit is updated by
                   OpenDEStateSpace::evaluateCollision() and OpenDEControlSpace::propagate(). A value of 1 implies that
                   there is no collision for which OpenDEEnvironment::isValidCollision() returns false. */
                STATE_COLLISION_VALUE_BIT = 1,
                /** \brief Index of bit in StateType::collision indicating whether it is known if a state is in valid or
                   not. Initially this is 0. The value of this bit is updated by OpenDEStateValidityChecker::isValid().
                   This bit is only used if the OpenDEStateValidityChecker is used. */
                STATE_VALIDITY_KNOWN_BIT = 2,
                /** \brief Index of bit in StateType::collision indicating whether a state is valid or not. Initially
                   the value of this field is unspecified. The value gains meaning (1 or 0) when
                   OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT becomes 1. The value of this bit is updated by
                   OpenDEEnvironment::isValid(). A value of 1 implies that a state is valid. This bit is only used if
                   the OpenDEStateValidityChecker is used. */
                STATE_VALIDITY_VALUE_BIT = 3,
            };

            /** \brief OpenDE State. This is a compound state that allows accessing the properties of the bodies the
             * state space is constructed for. */
            class StateType : public base::CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                /** \brief Get the position (x, y, z) of the body at index \e body */
                const double *getBodyPosition(unsigned int body) const
                {
                    return as<base::RealVectorStateSpace::StateType>(body * 4)->values;
                }

                /** \brief Get the position (x, y, z) of the body at index \e body */
                double *getBodyPosition(unsigned int body)
                {
                    return as<base::RealVectorStateSpace::StateType>(body * 4)->values;
                }

                /** \brief Get the quaternion of the body at index \e body */
                const base::SO3StateSpace::StateType &getBodyRotation(unsigned int body) const
                {
                    return *as<base::SO3StateSpace::StateType>(body * 4 + 3);
                }

                /** \brief Get the quaternion of the body at index \e body */
                base::SO3StateSpace::StateType &getBodyRotation(unsigned int body)
                {
                    return *as<base::SO3StateSpace::StateType>(body * 4 + 3);
                }

                /** \brief Get the linear velocity (x, y, z) of the body at index \e body */
                const double *getBodyLinearVelocity(unsigned int body) const
                {
                    return as<base::RealVectorStateSpace::StateType>(body * 4 + 1)->values;
                }

                /** \brief Get the linear velocity (x, y, z) of the body at index \e body */
                double *getBodyLinearVelocity(unsigned int body)
                {
                    return as<base::RealVectorStateSpace::StateType>(body * 4 + 1)->values;
                }

                /** \brief Get the angular velocity (x, y, z) of the body at index \e body */
                const double *getBodyAngularVelocity(unsigned int body) const
                {
                    return as<base::RealVectorStateSpace::StateType>(body * 4 + 2)->values;
                }

                /** \brief Get the angular velocity (x, y, z) of the body at index \e body */
                double *getBodyAngularVelocity(unsigned int body)
                {
                    return as<base::RealVectorStateSpace::StateType>(body * 4 + 2)->values;
                }

                /** \brief Flag containing information about state validity.

                    - BIT 0: (OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT)
                    - BIT 1: (OpenDEStateSpace::STATE_COLLISION_VALUE_BIT)
                    - BIT 2: (OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT)
                    - BIT 3: (OpenDEStateSpace::STATE_VALIDITY_VALUE_BIT) */
                mutable int collision{0};
            };

            /** \brief Construct a state space representing OpenDE states.

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
            OpenDEStateSpace(OpenDEEnvironmentPtr env, double positionWeight = 1.0, double linVelWeight = 0.5,
                             double angVelWeight = 0.5, double orientationWeight = 1.0);

            ~OpenDEStateSpace() override = default;

            /** \brief Get the OpenDE environment this state space corresponds to */
            const OpenDEEnvironmentPtr &getEnvironment() const
            {
                return env_;
            }

            /** \brief Get the number of bodies state is maintained for */
            unsigned int getNrBodies() const
            {
                return env_->stateBodies_.size();
            }

            /** \brief By default, the volume bounds enclosing the
              geometry of the environment are computed to include all
              objects in the spaces collision checking is performed
              (env.collisionSpaces_). The linear and angular velocity
              bounds are set as -1 to 1 for each dimension. */
            void setDefaultBounds();

            /** \brief Set the bounds for each of the position subspaces */
            void setVolumeBounds(const base::RealVectorBounds &bounds);

            /** \brief Set the bounds for each of the linear velocity subspaces */
            void setLinearVelocityBounds(const base::RealVectorBounds &bounds);

            /** \brief Set the bounds for each of the angular velocity subspaces */
            void setAngularVelocityBounds(const base::RealVectorBounds &bounds);

            /** \brief Read the parameters of the OpenDE bodies and store
                them in \e state. */
            virtual void readState(base::State *state) const;

            /** \brief Set the parameters of the OpenDE bodies to be the
                ones read from \e state.  The code will technically work if
                this function is called from multiple threads
                simultaneously, but the results are unpredictable. */
            virtual void writeState(const base::State *state) const;

            /** \brief This is a convenience function provided for
                optimization purposes. It checks whether a state
                satisfies its bounds. Typically, in the process of
                simulation the rotations remain valid (very slightly
                out of bounds), so there is no point in updating or
                checking them. This function checks all other bounds
                (position, linear and agular velocities) */
            bool satisfiesBoundsExceptRotation(const StateType *state) const;

            base::State *allocState() const override;
            void freeState(base::State *state) const override;
            void copyState(base::State *destination, const base::State *source) const override;
            void interpolate(const base::State *from, const base::State *to, double t,
                             base::State *state) const override;

            base::StateSamplerPtr allocDefaultStateSampler() const override;
            base::StateSamplerPtr allocStateSampler() const override;

            /** \brief Fill the OpenDEStateSpace::STATE_COLLISION_VALUE_BIT of StateType::collision member of a state,
               if unspecified.
                Return the value value of that bit. */
            virtual bool evaluateCollision(const base::State *state) const;

        protected:
            /** \brief Representation of the OpenDE parameters OMPL needs to plan */
            OpenDEEnvironmentPtr env_;
        };
    }
}

#endif
