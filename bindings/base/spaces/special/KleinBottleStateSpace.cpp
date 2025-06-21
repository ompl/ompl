#include <nanobind/nanobind.h>
#include "ompl/base/spaces/special/KleinBottleStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_KleinBottleStateSpace(nb::module_& m)
{
    // TODO [ob::KleinBottleStateSampler][IMPLEMENT]
    nb::class_<ob::KleinBottleStateSampler, ob::StateSampler>(m, "KleinBottleStateSampler")
        ;
    // TODO [ob::KleinBottleStateSpace][IMPLEMENT]
    nb::class_<ob::KleinBottleStateSpace, ob::CompoundStateSpace>(m, "KleinBottleStateSpace")
        ;
}
