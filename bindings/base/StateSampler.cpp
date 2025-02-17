#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSampler.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_StateSampler(nb::module_& m)
{
    nb::class_<ompl::base::StateSampler>(m, "StateSampler");
}
