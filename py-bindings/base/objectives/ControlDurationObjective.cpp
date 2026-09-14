#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/ControlDurationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/control/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

void ompl::binding::base::initObjectives_ControlDurationObjective(nb::module_ &m)
{
    nb::class_<ob::ControlDurationObjective, ob::OptimizationObjective>(m, "ControlDurationObjective")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("stateCost", &ob::ControlDurationObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::ControlDurationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("controlCost", &ob::ControlDurationObjective::controlCost, nb::arg("c"), nb::arg("steps"));
}
