#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/MinimaxObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_MinimaxObjective(nb::module_& m)
{
    // TODO [ob::MinimaxObjective][IMPLEMENT]
    nb::class_<ob::MinimaxObjective, ob::OptimizationObjective>(m, "MinimaxObjective");
}
