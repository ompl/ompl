#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
void ompl::binding::base::initObjectives_MaximizeMinClearanceObjective(nb::module_& m)
{
    // TODO [ob::MaximizeMinClearanceObjective][IMPLEMENT]
    nb::class_<ob::MaximizeMinClearanceObjective, ob::MinimaxObjective>(m, "MaximizeMinClearanceObjective");
}
