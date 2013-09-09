/* MorseStateValidityChecker.h */

#ifndef OMPL_EXTENSION_MORSE_STATE_VALIDITY_CHECKER_
#define OMPL_EXTENSION_MORSE_STATE_VALIDITY_CHECKER_

#include "ompl/control/SpaceInformation.h"
#include "ompl/extensions/morse/MorseStateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief The simplest state validity checker: all states are valid if
            they are within bounds */
        class MorseStateValidityChecker : public StateValidityChecker
        {
        public:

            /** \brief Constructor */
            MorseStateValidityChecker(const SpaceInformationPtr &si);

            /** \brief A state is always considered valid if it satisfies the bounds */
            virtual bool isValid(const State *state) const;

        protected:

            /** \brief The corresponding MORSE state space */
            MorseStateSpace *mss_;
        };
    }
}
#endif
