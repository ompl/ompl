/* MorseStatePropagator.h */

#ifndef OMPL_EXTENSION_MORSE_STATE_PROPAGATOR_
#define OMPL_EXTENSION_MORSE_STATE_PROPAGATOR_

#include "ompl/control/SpaceInformation.h"
#include "ompl/extensions/morse/MorseEnvironment.h"

namespace ompl
{

    namespace control
    {

        /** \brief State propagation with MORSE. Only forward
            propagation is possible.

            At every propagation step, controls are applied using
            MorseEnvironment::applyControl(), and then \b worldStep()
            is called.*/
        class MorseStatePropagator : public StatePropagator
        {
        public:

            /** \brief Construct a representation of MORSE state propagator.
                If \e si->getStateSpace() does not cast to an
                MorseStateSpace, an exception is thrown. */
            MorseStatePropagator(const SpaceInformationPtr &si);

            virtual ~MorseStatePropagator(void)
            {
            }

            /** \brief Get the MORSE environment this state propagator operates on */
            const MorseEnvironmentPtr& getEnvironment(void) const
            {
                return env_;
            }

            virtual bool canPropagateBackward(void) const;

            virtual void propagate(const base::State *state, const Control* control, const double duration, base::State *result) const;

        protected:

            /** \brief The MORSE environment this state propagator operates on */
            MorseEnvironmentPtr env_;

        };
    }

}

#endif
