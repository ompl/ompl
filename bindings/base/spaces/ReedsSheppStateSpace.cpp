#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "../init.hh"

namespace nb  = nanobind;
namespace ob  = ompl::base;

void ompl::binding::base::initSpaces_ReedsSheppStateSpace(nb::module_& m) {
    // TODO [ob::ReedsSheppStateSpace][IMPLEMENT]
    nb::class_<ob::ReedsSheppStateSpace::ReedsSheppPath>(m, "ReedsSheppPath");
    // TODO [ob::ReedsSheppStateSpace::ReedsSheppPath][IMPLEMENT]
    nb::class_<ob::ReedsSheppStateSpace, ob::SE2StateSpace>(m, "ReedsSheppStateSpace");
    // TODO [ob::ReedsSheppStateSpace][IMPLEMENT]
    nb::class_<ob::ReedsSheppMotionValidator, ob::MotionValidator>(m, "ReedsSheppMotionValidator");

}
