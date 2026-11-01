#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/est/EST.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersEst_EST(nb::module_ &m)
{
    nb::class_<og::EST, ob::Planner>(m, "EST")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::EST>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::EST>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::EST &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::EST::clear)
        .def("setup", &og::EST::setup)
        .def("setGoalBias", &og::EST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::EST::getGoalBias)
        .def("setRange", &og::EST::setRange, nb::arg("distance"))
        .def("getRange", &og::EST::getRange);
}
