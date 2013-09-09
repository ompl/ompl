/* MorseStateValidityChecker.cpp */

#include "ompl/extensions/morse/MorseStateValidityChecker.h"
#include "ompl/util/Exception.h"

ompl::base::MorseStateValidityChecker::MorseStateValidityChecker(const SpaceInformationPtr &si) : StateValidityChecker(si)
{
    if (!dynamic_cast<MorseStateSpace*>(si->getStateSpace().get()))
        throw Exception("Cannot create state validity checking for MORSE without MORSE state space");
    mss_ = si->getStateSpace()->as<MorseStateSpace>();
}

bool ompl::base::MorseStateValidityChecker::isValid(const State *state) const
{
    return mss_->satisfiesBounds(state->as<MorseStateSpace::StateType>());
}
