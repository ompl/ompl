#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersKpiece_KPIECE1(nb::module_ &m)
{
    nb::class_<oc::KPIECE1, ob::Planner>(m, "KPIECE1").def(nb::init<const oc::SpaceInformationPtr &>());
}
