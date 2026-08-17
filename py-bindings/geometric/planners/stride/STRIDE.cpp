#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/stride/STRIDE.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersStride_STRIDE(nb::module_ &m)
{
    nb::class_<og::STRIDE, ob::Planner>(m, "STRIDE")
        .def(nb::init<const ob::SpaceInformationPtr &, bool, unsigned int, unsigned int, unsigned int, unsigned int,
                      double>(),
             nb::arg("si"), nb::arg("useProjectedDistance") = false, nb::arg("degree") = 16, nb::arg("minDegree") = 12,
             nb::arg("maxDegree") = 18, nb::arg("maxNumPtsPerLeaf") = 6, nb::arg("estimatedDimension") = 0.0)
        .def("solve",
             [](og::STRIDE &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def(
            "getPlannerData", [](const og::STRIDE &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::STRIDE::clear)
        .def("setup", &og::STRIDE::setup)
        .def("setGoalBias", &og::STRIDE::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::STRIDE::getGoalBias)
        .def("setUseProjectedDistance", &og::STRIDE::setUseProjectedDistance, nb::arg("useProjectedDistance"))
        .def("getUseProjectedDistance", &og::STRIDE::getUseProjectedDistance)
        .def("setDegree", &og::STRIDE::setDegree, nb::arg("degree"))
        .def("getDegree", &og::STRIDE::getDegree)
        .def("setMinDegree", &og::STRIDE::setMinDegree, nb::arg("minDegree"))
        .def("getMinDegree", &og::STRIDE::getMinDegree)
        .def("setMaxDegree", &og::STRIDE::setMaxDegree, nb::arg("maxDegree"))
        .def("getMaxDegree", &og::STRIDE::getMaxDegree)
        .def("setMaxNumPtsPerLeaf", &og::STRIDE::setMaxNumPtsPerLeaf, nb::arg("maxNumPtsPerLeaf"))
        .def("getMaxNumPtsPerLeaf", &og::STRIDE::getMaxNumPtsPerLeaf)
        .def("setEstimatedDimension", &og::STRIDE::setEstimatedDimension, nb::arg("estimatedDimension"))
        .def("getEstimatedDimension", &og::STRIDE::getEstimatedDimension)
        .def("setRange", &og::STRIDE::setRange, nb::arg("distance"))
        .def("getRange", &og::STRIDE::getRange)
        .def("setMinValidPathFraction", &og::STRIDE::setMinValidPathFraction, nb::arg("fraction"))
        .def("getMinValidPathFraction", &og::STRIDE::getMinValidPathFraction)
        .def("setProjectionEvaluator",
             static_cast<void (og::STRIDE::*)(const ob::ProjectionEvaluatorPtr &)>(&og::STRIDE::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::STRIDE::*)(const std::string &)>(&og::STRIDE::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &og::STRIDE::getProjectionEvaluator, nb::rv_policy::reference_internal);
}
