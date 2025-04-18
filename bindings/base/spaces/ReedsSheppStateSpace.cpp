#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "../init.hh"

namespace nb  = nanobind;
namespace ob  = ompl::base;

void ompl::binding::base::initSpaces_ReedsSheppStateSpace(nb::module_& m) {
    //
    // Bind the inner ReedsSheppPath class
    //
    nb::class_<ob::ReedsSheppStateSpace::ReedsSheppPath>(m, "ReedsSheppPath");

    //
    // Bind the ReedsSheppStateSpace itself
    //
    nb::class_<ob::ReedsSheppStateSpace, ob::SE2StateSpace>(m, "ReedsSheppStateSpace");

    //
    // Bind the associated motion validator
    //
    nb::class_<ob::ReedsSheppMotionValidator, ob::MotionValidator>(m, "ReedsSheppMotionValidator");

}
