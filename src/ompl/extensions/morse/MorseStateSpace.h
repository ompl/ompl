/* MorseStateSpace.h */

#ifndef OMPL_EXTENSION_MORSE_STATE_SPACE_
#define OMPL_EXTENSION_MORSE_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/extensions/morse/MorseEnvironment.h"

namespace ompl
{
    namespace base
    {
        
        /** \brief State space representing MORSE states */
        class MorseStateSpace : public CompoundStateSpace
        {
        public:

            /** \brief MORSE State. This is a compound state that allows accessing the properties of the bodies the state space is constructed for. */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType(void) : CompoundStateSpace::StateType(), validCollision(true)
                {
                }

                /** \brief Flag containing information about state validity */
                mutable bool validCollision;

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
                \param linVelWeight the weight to pass to CompoundStateSpace::addSubspace() for linear velocity subspaces
                \param angVelWeight the weight to pass to CompoundStateSpace::addSubspace() for angular velocity subspaces
                \param orientationWeight the weight to pass to CompoundStateSpace::addSubspace() for orientation subspaces
            */
            MorseStateSpace(const MorseEnvironmentPtr &env,
                             double positionWeight = 1.0, double linVelWeight = 0.5,
                             double angVelWeight = 0.5, double orientationWeight = 1.0);

            virtual ~MorseStateSpace(void)
            {
            }

            /** \brief Get the MORSE environment this state space corresponds to */
            const MorseEnvironmentPtr& getEnvironment(void) const
            {
                return env_;
            }

            /** \brief Get the number of bodies state is maintained for */
            unsigned int getNrBodies(void) const
            {
                return env_->rigidBodies_;
            }

            /** \brief Set the bounds given by MorseEnvironment */
            void setBounds(void);

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

            /** \brief This is a convenience function provided for
                optimization purposes. It checks whether a state
                satisfies its bounds. Typically, in the process of
                simulation the rotations remain valid (very slightly
                out of bounds), so there is no point in updating or
                checking them. This function checks all other bounds
                (position, linear and angular velocities) */
            bool satisfiesBounds(const State *state) const;

            State* allocState(void) const;
            void freeState(State *state) const;
            void copyState(State *destination, const State *source) const;
            void interpolate(const State *from, const State *to, const double t, State *state) const;

            StateSamplerPtr allocDefaultStateSampler(void) const;
            StateSamplerPtr allocStateSampler(void) const;

        protected:

            /** \brief Representation of the MORSE parameters OMPL needs to plan */
            MorseEnvironmentPtr env_;
        };
    }
}


#endif
