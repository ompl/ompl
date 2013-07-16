/* MorseSimpleSetup.h */

#ifndef OMPL_EXTENSION_MORSE_SIMPLE_SETUP_
#define OMPL_EXTENSION_MORSE_SIMPLE_SETUP_

#include "ompl/control/SimpleSetup.h"
#include "ompl/extensions/morse/MorseStateValidityChecker.h"
#include "ompl/extensions/morse/MorseStatePropagator.h"
#include "ompl/extensions/morse/MorseControlSpace.h"

namespace ompl
{

    namespace control
    {

        /** \brief Create the set of classes typically needed to solve a
            control problem when forward propagation is computed with OpenDE. */
        class MorseSimpleSetup : public SimpleSetup
        {
        public:

            /** \brief The control space is assumed to be
                MorseControlSpace. The state space is assumed to
                be MorseStateSpace. Constructor only needs the MORSE
                environment. */
            explicit
            MorseSimpleSetup(const base::MorseEnvironmentPtr &env);

            virtual ~MorseSimpleSetup(void)
            {
            }

            /** \brief Get the MORSE environment associated to the state and control spaces */
            const base::MorseEnvironmentPtr& getEnvironment(void) const
            {
                return getStateSpace()->as<base::MorseStateSpace>()->getEnvironment();
            }

            /** \brief Get the current MORSE state (read parameters from MORSE bodies) */
            base::ScopedState<base::MorseStateSpace> getCurrentState(void) const;

            /** \brief Set the current MORSE state (set parameters for MORSE bodies) */
            void setCurrentState(const base::ScopedState<> &state);

            /** \brief Set the current MORSE state (set parameters for MORSE bodies) */
            void setCurrentState(const base::State *state);

            void setup(void);
            
            base::PlannerStatus solve(double time = 1.0);
            
            /** \brief Set the MORSE world to the states that are
                contained in a given path, sequentially. */
            void playPath(const base::PathPtr &path) const;

            /** \brief Call playPath() on the solution path, if one is available */
            void playSolutionPath(void) const;

            /** \brief Simulate the MORSE environment forward for \e steps simulation steps, using the control \e control.
                Construct a path representing this action. */
            base::PathPtr simulateControl(const double* control, unsigned int steps) const;

            /** \brief Simulate the MORSE environment forward for \e steps simulation steps, using the control \e control.
                Construct a path representing this action. */
            base::PathPtr simulateControl(const Control* control, unsigned int steps) const;

            /** \brief Simulate the MORSE environment forward for \e
                steps simulation steps, using the null control
                (ompl::control::ControlSpace::nullControl()).
                Construct a path representing this action. */
            base::PathPtr simulate(unsigned int steps) const;

        private:

            void useEnvParams(void);

        };
    }

}
#endif
