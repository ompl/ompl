/* MorseGoal.h */

#ifndef OMPL_EXTENSION_MORSE_GOAL_
#define OMPL_EXTENSION_MORSE_GOAL_

#include "ompl/base/Goal.h"
#include "ompl/base/SpaceInformation.h"

namespace ompl
{

    namespace base
    {
    
        /*
           This dummy class exists so that the planner must call MorseGoal::isSatisfied(const State*, double*),
           forcing the use of base::Goal::isSatisfied(const State*, double*), which in turn calls the pure
           virtual function base::Goal::isSatisfied(const State*) that is implemented here, calling the pure virtual
           function isSatisfied_Py, which is left to be implemented in Python. If we don't use MorseGoal, and try to
           make a Python goal inherit directly from base::Goal, then the bindings will try to call isSatisfied(
           const State*, double*) on the Python goal, which is problematic because Python can't handle C++ double*.
       */  
        class MorseGoal : public Goal
        {
        public:
            MorseGoal(SpaceInformationPtr si)
                : Goal(si)
            {
            }
            
            virtual bool isSatisfied_Py(const State *state) const = 0;
            
            bool isSatisfied(const State* state) const;
        };
    }
}
#endif


