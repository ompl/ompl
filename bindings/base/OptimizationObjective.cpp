#include <nanobind/nanobind.h>
#include "ompl/base/OptimizationObjective.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_OptimizationObjective(nb::module_& m)
{
    nb::class_<ompl::base::OptimizationObjective>(m, "OptimizationObjective");
}
