#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/est/ProjEST.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersEst_ProjEST(nb::module_ &m)
{
    nb::class_<og::ProjEST, ob::Planner>(m, "ProjEST")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::ProjEST &self, nb::object what)
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
            "getPlannerData", [](const og::ProjEST &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::ProjEST::clear)
        .def("setup", &og::ProjEST::setup)
        .def("setGoalBias", &og::ProjEST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::ProjEST::getGoalBias)
        .def("setRange", &og::ProjEST::setRange, nb::arg("distance"))
        .def("getRange", &og::ProjEST::getRange)
        .def("setProjectionEvaluator",
             static_cast<void (og::ProjEST::*)(const ob::ProjectionEvaluatorPtr &)>(
                 &og::ProjEST::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::ProjEST::*)(const std::string &)>(&og::ProjEST::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &og::ProjEST::getProjectionEvaluator, nb::rv_policy::reference_internal);
}
