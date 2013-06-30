/* MorseGoal.cpp */

#include "ompl/extensions/morse/MorseGoal.h"

bool ompl::base::MorseGoal::isSatisfied(const State *state) const
{
    return isSatisfied_Py(state);
}


