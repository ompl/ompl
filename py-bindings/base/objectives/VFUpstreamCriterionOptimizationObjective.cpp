#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::base::initObjectives_VFUpstreamCriterionOptimizationObjective(nb::module_ &m)
{
    nb::class_<ob::VFUpstreamCriterionOptimizationObjective, ob::OptimizationObjective>(
        m, "VFUpstreamCriterionOptimizationObjective")
        .def(nb::init<const ob::SpaceInformationPtr &, og::VFRRT::VectorField>(), nb::arg("si"), nb::arg("vf"))
        .def("isSatisfied", &ob::VFUpstreamCriterionOptimizationObjective::isSatisfied, nb::arg("c"))
        .def("stateCost", &ob::VFUpstreamCriterionOptimizationObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::VFUpstreamCriterionOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("isSymmetric", &ob::VFUpstreamCriterionOptimizationObjective::isSymmetric);
}
