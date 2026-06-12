#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    og::VFRRT::VectorField wrapVectorField(og::VFRRT::VectorField vf)
    {
        return [vf = std::move(vf)](const ob::State *state) -> Eigen::VectorXd
        {
            nb::gil_scoped_acquire gil;
            return vf(state);
        };
    }
}  // namespace

void ompl::binding::base::initObjectives_VFUpstreamCriterionOptimizationObjective(nb::module_ &m)
{
    nb::class_<ob::VFUpstreamCriterionOptimizationObjective, ob::OptimizationObjective>(
        m, "VFUpstreamCriterionOptimizationObjective")
        .def(
            "__init__",
            [](ob::VFUpstreamCriterionOptimizationObjective *self, const ob::SpaceInformationPtr &si,
               og::VFRRT::VectorField vf)
            { new (self) ob::VFUpstreamCriterionOptimizationObjective(si, wrapVectorField(std::move(vf))); },
            nb::arg("si"), nb::arg("vf"))
        .def("isSatisfied", &ob::VFUpstreamCriterionOptimizationObjective::isSatisfied, nb::arg("c"))
        .def("stateCost", &ob::VFUpstreamCriterionOptimizationObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::VFUpstreamCriterionOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("isSymmetric", &ob::VFUpstreamCriterionOptimizationObjective::isSymmetric);
}
