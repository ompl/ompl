#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_DubinsStateSpace(nb::module_ &m)
{
    // TODO
}
