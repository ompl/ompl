#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_LazyRRT(nb::module_ &m)
{
    nb::class_<og::LazyRRT, ob::Planner>(m, "LazyRRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::LazyRRT>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::LazyRRT>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::LazyRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LazyRRT::clear)
        .def("setup", &og::LazyRRT::setup)
        .def("setGoalBias", &og::LazyRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::LazyRRT::getGoalBias)
        .def("setRange", &og::LazyRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::LazyRRT::getRange);
}
