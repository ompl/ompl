#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/ValidStateSampler.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_ValidStateSampler(nb::module_& m)
{
    // TODO [ompl::base::ValidStateSampler][IMPLEMENT]
    nb::class_<ompl::base::ValidStateSampler>(m, "ValidStateSampler");
}
