#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/VFMechanicalWorkOptimizationObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_VFMechanicalWorkOptimizationObjective(nb::module_& m)
{
    // TODO [ob::VFMechanicalWorkOptimizationObjective][IMPLEMENT]
    nb::class_<ob::VFMechanicalWorkOptimizationObjective, ob::MechanicalWorkOptimizationObjective>(m, "VFMechanicalWorkOptimizationObjective");
}
