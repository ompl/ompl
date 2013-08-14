/* MorseGoal.cpp */

#include "ompl/extensions/morse/MorseGoal.h"

#include "ompl/util/Console.h"

bool ompl::base::MorseGoal::isSatisfied(const State *state) const
{
    return isSatisfied_Py(state);
}
            
bool ompl::base::MorseGoal::isSatisfied(const State *state, double *distance) const
{
    bool sat = isSatisfied_Py(state);
    if (distance != NULL)
        *distance = distance_;
    return sat;
}

