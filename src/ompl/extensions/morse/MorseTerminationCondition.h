/* MorseTerminationCondition.h */

#ifndef OMPL_EXTENSION_MORSE_TERMINATION_CONDITION_
#define OMPL_EXTENSION_MORSE_TERMINATION_CONDITION_

#include "ompl/extensions/morse/MorseEnvironment.h"
#include "ompl/base/PlannerTerminationCondition.h"

namespace ompl
{
    namespace base
    {
        
        /** This class represents a termination condition for the planner that
            only terminates if the user shuts down the MORSE simulation */
        class MorseTerminationCondition : public PlannerTerminationCondition
        {
        public:
            
            /** \brief The representation of the MORSE simulation environment */
            const MorseEnvironmentPtr env_;
            
            MorseTerminationCondition(const MorseEnvironmentPtr env) :
                PlannerTerminationCondition(NULL), env_(env)
            {
            }
            
            ~MorseTerminationCondition(void)
            {
            }
            
            /** \brief Return true if the simulation is still running */
            bool eval(void) const;
        };
    }
}

#endif

