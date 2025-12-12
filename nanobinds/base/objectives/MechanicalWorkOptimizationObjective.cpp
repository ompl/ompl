#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
void ompl::binding::base::initObjectives_MechanicalWorkOptimizationObjective(nb::module_& m)
{
    // TODO [ob::MechanicalWorkOptimizationObjective][IMPLEMENT]
    // TODO [ob::MechanicalWorkOptimizationObjective][TRAMPOLINE]
    nb::class_<ob::MechanicalWorkOptimizationObjective, ob::OptimizationObjective>(m, "MechanicalWorkOptimizationObjective");
}
