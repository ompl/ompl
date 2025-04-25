#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/OptimizationObjective.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_OptimizationObjective(nb::module_& m)
{
    // TODO [ob::OptimizationObjective][IMPLEMENT]
    // TODO [ob::OptimizationObjective][TRAMPOLINE]
    nb::class_<ob::OptimizationObjective>(m, "OptimizationObjective");

    // TODO [ob::MultiOptimizationObjective][IMPLEMENT]
    nb::class_<ob::MultiOptimizationObjective, ob::OptimizationObjective>(m, "MultiOptimizationObjective");
}
