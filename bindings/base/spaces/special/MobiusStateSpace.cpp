#include <nanobind/nanobind.h>
#include "ompl/base/spaces/special/MobiusStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_MobiusStateSpace(nb::module_& m)
{
    // TODO [ob::MobiusStateSampler][IMPLEMENT]
    nb::class_<ob::MobiusStateSpace, ob::CompoundStateSpace>(m, "MobiusStateSpace")
        ;
}
