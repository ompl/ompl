/* MorseGoal.h */

#ifndef OMPL_EXTENSION_MORSE_GOAL_
#define OMPL_EXTENSION_MORSE_GOAL_

#include "ompl/base/Goal.h"
#include "ompl/base/SpaceInformation.h"

#include <limits>

namespace ompl
{

    namespace base
    {
    
        /** \brief This is a goal class that is more amenable to Python  */
        class MorseGoal : public Goal
        {
        public:
            MorseGoal(SpaceInformationPtr si)
                : Goal(si), distance_(std::numeric_limits<double>::max())
            {
            }
            
            /** \brief Where MORSE will store the distance to goal during an isSatisfied() call */
            mutable double distance_;
            
            /** \brief Return true if \e state satisfies the goal */
            bool isSatisfied(const State *state) const;
            
            /** \brief Return true if \e state satisfies the goal, and store the distance
                to the goal in \e distance */
            bool isSatisfied(const State *state, double *distance) const;
            
            /** \brief To be implemented in Python; behaves like isSatisfied(state, &distance_) */
            virtual bool isSatisfied_Py(const State *state) const = 0;
        };
    }
}

#endif


