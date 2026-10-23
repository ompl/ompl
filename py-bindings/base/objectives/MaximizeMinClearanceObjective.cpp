#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_MaximizeMinClearanceObjective(nb::module_ &m)
{
    nb::class_<ob::MaximizeMinClearanceObjective, ob::MinimaxObjective>(m, "MaximizeMinClearanceObjective")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("stateCost", &ob::MaximizeMinClearanceObjective::stateCost, nb::arg("s"))
        .def("isCostBetterThan", &ob::MaximizeMinClearanceObjective::isCostBetterThan, nb::arg("c1"), nb::arg("c2"))
        .def("identityCost", &ob::MaximizeMinClearanceObjective::identityCost)
        .def("infiniteCost", &ob::MaximizeMinClearanceObjective::infiniteCost);
}
