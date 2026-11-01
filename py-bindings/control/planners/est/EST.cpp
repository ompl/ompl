#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include "ompl/control/planners/est/EST.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersEst_EST(nb::module_ &m)
{
    nb::class_<ompl::control::EST, ob::Planner>(m, "EST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<oc::EST>(), nb::arg("terminationCondition"))
        .def("solve", ompl::binding::solveFnBinding<oc::EST>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def("setGoalBias", &oc::EST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::EST::getGoalBias)
        .def("setRange", &oc::EST::setRange, nb::arg("distance"))
        .def("getRange", &oc::EST::getRange)
        .def("setProjectionEvaluator",
             nb::overload_cast<const ob::ProjectionEvaluatorPtr &>(&oc::EST::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator", nb::overload_cast<const std::string &>(&oc::EST::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &oc::EST::getProjectionEvaluator);
}
