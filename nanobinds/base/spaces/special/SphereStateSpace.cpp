#include <nanobind/nanobind.h>
#include "ompl/base/spaces/special/SphereStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_SphereStateSpace(nb::module_& m)
{
    // TODO [ob::SphereStateSampler][IMPLEMENT]
    nb::class_<ob::SphereStateSampler, ob::StateSampler>(m, "SphereStateSampler")
        ;
    // TODO [ob::SphereStateSpace][IMPLEMENT]
    nb::class_<ob::SphereStateSpace, ob::CompoundStateSpace>(m, "SphereStateSpace")
        ;
    // TODO [ob::SphereStateSpace::StateType][IMPLEMENT]
    nb::class_<ob::SphereStateSpace::StateType, ob::CompoundStateSpace::StateType>(m, "SphereStateSpaceStateType")
        ;
}
