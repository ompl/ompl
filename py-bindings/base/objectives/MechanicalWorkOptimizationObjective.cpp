#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_MechanicalWorkOptimizationObjective(nb::module_ &m)
{
    nb::class_<ob::MechanicalWorkOptimizationObjective, ob::OptimizationObjective>(m,
                                                                                   "MechanicalWorkOptimizationObjective")
        .def(nb::init<const ob::SpaceInformationPtr &, double>(), nb::arg("si"), nb::arg("pathLengthWeight") = 0.00001)
        .def("getPathLengthWeight", &ob::MechanicalWorkOptimizationObjective::getPathLengthWeight)
        .def("stateCost", &ob::MechanicalWorkOptimizationObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::MechanicalWorkOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"));
}
