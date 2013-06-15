/* MorseStatePropagator.cpp */

#include "ompl/extensions/morse/MorseStatePropagator.h"
#include "ompl/extensions/morse/MorseStateSpace.h"
#include "ompl/extensions/morse/MorseControlSpace.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"

ompl::control::MorseStatePropagator::MorseStatePropagator(const SpaceInformationPtr &si) : StatePropagator(si)
{
    if (base::MorseStateSpace *mss = dynamic_cast<base::MorseStateSpace*>(si->getStateSpace().get()))
        env_ = mss->getEnvironment();
    else
        throw Exception("MORSE State Space needed for MorseStatePropagator");
}

void ompl::control::MorseStatePropagator::propagate(const base::State *state, const Control* control, const double duration, base::State *result) const
{
    env_->mutex_.lock();

    // place the MORSE world at the start state
    si_->getStateSpace()->as<base::MorseStateSpace>()->writeState(state);

    // apply the controls
    env_->applyControl(control->as<RealVectorControlSpace::ControlType>()->values);

    // propagate one step forward
    env_->worldStep(duration);

    // read the final state from the MORSE world
    si_->getStateSpace()->as<base::MorseStateSpace>()->readState(result);

    env_->mutex_.unlock();
}

bool ompl::control::MorseStatePropagator::canPropagateBackward(void) const
{
    return false;
}
