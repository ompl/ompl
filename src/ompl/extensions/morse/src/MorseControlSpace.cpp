/* MorseControlSpace.cpp */

#include "ompl/extensions/morse/MorseControlSpace.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"

/// @cond IGNORE
namespace ompl
{
    const base::MorseEnvironmentPtr& getMorseStateSpaceEnvironmentWithCheck(const base::StateSpacePtr &space)
    {
        if (!dynamic_cast<base::MorseStateSpace*>(space.get()))
            throw Exception("MORSE State Space needed for creating MORSE Control Space");
        return space->as<base::MorseStateSpace>()->getEnvironment();
    }
}
/// @endcond

ompl::control::MorseControlSpace::MorseControlSpace(const base::StateSpacePtr &stateSpace) :
    RealVectorControlSpace(stateSpace, getMorseStateSpaceEnvironmentWithCheck(stateSpace)->wi_.conDim_)
{
    setName("Morse" + getName());
    type_ = CONTROL_SPACE_TYPE_COUNT + 1;
    base::RealVectorBounds bounds(dimension_);
    getEnvironment()->getControlBounds(bounds.low, bounds.high);
    setBounds(bounds);
}
