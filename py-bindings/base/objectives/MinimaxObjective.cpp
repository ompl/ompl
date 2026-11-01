#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_MinimaxObjective(nb::module_ &m)
{
    nb::class_<ob::MinimaxObjective, ob::OptimizationObjective>(m, "MinimaxObjective")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("stateCost", &ob::MinimaxObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::MinimaxObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("combineCosts", &ob::MinimaxObjective::combineCosts, nb::arg("c1"), nb::arg("c2"));
}
