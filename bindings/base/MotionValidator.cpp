#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/MotionValidator.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_MotionValidator(nb::module_& m)
{
    nb::class_<ompl::base::MotionValidator>(m, "MotionValidator");
}
