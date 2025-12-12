#include <nanobind/nanobind.h>
#include "ompl/base/spaces/special/TorusStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_TorusStateSpace(nb::module_& m)
{
    // TODO [ob::TorusStateSampler][IMPLEMENT]
    nb::class_<ob::TorusStateSampler, ob::StateSampler>(m, "TorusStateSampler")
        ;
    // TODO [ob::TorusStateSpace][IMPLEMENT]
    nb::class_<ob::TorusStateSpace, ob::CompoundStateSpace>(m, "TorusStateSpace")
        ;
    // TODO [ob::TorusStateSpace::StateType][IMPLEMENT]
    nb::class_<ob::TorusStateSpace::StateType, ob::CompoundStateSpace::StateType>(m, "TorusStateSpaceStateType")
        ;
}
