#include <nanobind/nanobind.h>
#include "ompl/control/PlannerData.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_PlannerData(nb::module_ &m)
{
    nb::class_<oc::PlannerDataEdgeControl>(m, "PlannerDataEdgeControl");
    nb::class_<oc::PlannerData>(m, "PlannerData");
}
