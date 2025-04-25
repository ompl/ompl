#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_StateCostIntegralObjective(nb::module_& m)
{
    // TODO [ob::StateCostIntegralObjective][IMPLEMENT]
    nb::class_<ob::StateCostIntegralObjective, ob::OptimizationObjective>(m, "StateCostIntegralObjective");
}
