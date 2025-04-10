#include <nanobind/nanobind.h>
#include "ompl/base/ValidStateSampler.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_ValidStateSampler(nb::module_& m)
{
    nb::class_<ompl::base::ValidStateSampler>(m, "ValidStateSampler");
}
