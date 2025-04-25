#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_VFUpstreamCriterionOptimizationObjective(nb::module_& m)
{
    // TODO [ob::VFUpstreamCriterionOptimizationObjective][IMPLEMENT]
    nb::class_<ob::VFUpstreamCriterionOptimizationObjective, ob::OptimizationObjective>(m, "VFUpstreamCriterionOptimizationObjective");
}
