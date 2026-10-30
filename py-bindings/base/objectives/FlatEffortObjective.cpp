#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/FlatEffortObjective.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_FlatEffortObjective(nb::module_ &m)
{
    nb::class_<ob::FlatEffortObjective, ob::OptimizationObjective>(m, "FlatEffortObjective")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("stateCost", &ob::FlatEffortObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::FlatEffortObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("motionCostHeuristic", &ob::FlatEffortObjective::motionCostHeuristic, nb::arg("s1"), nb::arg("s2"))
        .def("motionCostBestEstimate", &ob::FlatEffortObjective::motionCostBestEstimate, nb::arg("s1"), nb::arg("s2"));
}
