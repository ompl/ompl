/* MorseTerminationCondition.h */

#ifndef OMPL_EXTENSION_MORSE_TERMINATION_CONDITION_
#define OMPL_EXTENSION_MORSE_TERMINATION_CONDITION_

#include "ompl/extensions/morse/MorseEnvironment.h"
#include "ompl/base/PlannerTerminationCondition.h"

namespace ompl
{
    namespace base
    {

        class MorseTerminationCondition : public PlannerTerminationCondition
        {
        public:
            
            const MorseEnvironmentPtr env_;
            
            MorseTerminationCondition(const MorseEnvironmentPtr env) :
                PlannerTerminationCondition(NULL), env_(env)
            {
            }
            
            ~MorseTerminationCondition(void)
            {
            }
            
            bool eval(void) const;
        };
    }
}

#endif

