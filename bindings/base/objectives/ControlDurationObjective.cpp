#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/ControlDurationObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_ControlDurationObjective(nb::module_& m)
{
    // TODO [ob::ControlDurationObjective][IMPLEMENT]
    nb::class_<ob::ControlDurationObjective, ob::OptimizationObjective>(m, "ControlDurationObjective"); 
}
