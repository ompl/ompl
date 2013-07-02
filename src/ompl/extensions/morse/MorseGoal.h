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
    
        class MorseGoal : public Goal
        {
        public:
            MorseGoal(SpaceInformationPtr si)
                : Goal(si), distance_(std::numeric_limits<double>::max())
            {
            }
            
            mutable double distance_;
            
            bool isSatisfied(const State *state) const;
            
            virtual bool isSatisfied_Py(const State *state) const = 0;
            
            bool isSatisfied(const State *state, double *distance) const;
            
        };
    }
}
#endif


