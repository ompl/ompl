#include <nanobind/nanobind.h>
#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_TangentBundleStateSpace(nb::module_& m)
{
    // TODO [ob::TangentBundleStateSpace][IMPLEMENT]
    nb::class_<ob::TangentBundleStateSpace, ob::AtlasStateSpace>(m, "TangentBundleStateSpace")
        ;
}
