#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/MotionValidator.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_MotionValidator(nb::module_& m)
{
    // TODO [ob::MotionValidator][IMPLEMENT]
    nb::class_<ob::MotionValidator>(m, "MotionValidator");
}
