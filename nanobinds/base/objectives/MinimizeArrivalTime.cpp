#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/objectives/MinimizeArrivalTime.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_MinimizeArrivalTime(nb::module_& m)
{
    // TODO [ob::MinimizeArrivalTime][IMPLEMENT]
    nb::class_<ob::MinimizeArrivalTime, ob::OptimizationObjective>(m, "MinimizeArrivalTime");
}
