#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_PathLengthOptimizationObjective(nb::module_& m)
{
    // TODO [ob::PathLengthOptimizationObjective][IMPLEMENT]
    nb::class_<ob::PathLengthOptimizationObjective, ob::OptimizationObjective>(m, "PathLengthOptimizationObjective");
}
