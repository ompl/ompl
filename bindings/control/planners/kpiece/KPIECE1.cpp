#include <nanobind/nanobind.h>
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersKpiece_KPIECE1(nb::module_& m)
{
    // TODO [oc::KPIECE1][IMPLEMENT]
    nb::class_<oc::KPIECE1, ob::Planner>(m, "KPIECE1")
        ;
}
