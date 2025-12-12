#include <nanobind/nanobind.h>
#include "ompl/control/PlannerData.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_PlannerData(nb::module_& m)
{
    // TODO [oc::PlannerDataEdgeControl][IMPLEMENT]
    nb::class_<oc::PlannerDataEdgeControl>(m, "PlannerDataEdgeControl")
        ;
    // TODO [oc::PlannerData][IMPLEMENT]
    nb::class_<oc::PlannerData>(m, "PlannerData")
        ;
}

