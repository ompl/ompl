#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/objectives/VFMechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::base::initObjectives_VFMechanicalWorkOptimizationObjective(nb::module_ &m)
{
    nb::class_<ob::VFMechanicalWorkOptimizationObjective, ob::MechanicalWorkOptimizationObjective>(
        m, "VFMechanicalWorkOptimizationObjective")
        .def(nb::init<const ob::SpaceInformationPtr &, og::VFRRT::VectorField>(), nb::arg("si"), nb::arg("vf"))
        .def("isSatisfied", &ob::VFMechanicalWorkOptimizationObjective::isSatisfied, nb::arg("c"))
        .def("motionCost", &ob::VFMechanicalWorkOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("isSymmetric", &ob::VFMechanicalWorkOptimizationObjective::isSymmetric);
}
