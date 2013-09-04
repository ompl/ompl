/* MorseTerminationCondition.cpp */

#include "ompl/extensions/morse/MorseTerminationCondition.h"

bool ompl::base::MorseTerminationCondition::eval(void) const
{
    return env_->simRunning_;
}
