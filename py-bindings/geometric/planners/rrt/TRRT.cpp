#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_TRRT(nb::module_ &m)
{
    nb::class_<og::TRRT, ob::Planner>(m, "TRRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::TRRT>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::TRRT>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::TRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::TRRT::clear)
        .def("setup", &og::TRRT::setup)
        .def("setGoalBias", &og::TRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::TRRT::getGoalBias)
        .def("setRange", &og::TRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::TRRT::getRange)
        .def("setTempChangeFactor", &og::TRRT::setTempChangeFactor, nb::arg("factor"))
        .def("getTempChangeFactor", &og::TRRT::getTempChangeFactor)
        .def("setCostThreshold", &og::TRRT::setCostThreshold, nb::arg("maxCost"))
        .def("getCostThreshold", &og::TRRT::getCostThreshold)
        .def("setInitTemperature", &og::TRRT::setInitTemperature, nb::arg("initTemperature"))
        .def("getInitTemperature", &og::TRRT::getInitTemperature)
        .def("setFrontierThreshold", &og::TRRT::setFrontierThreshold, nb::arg("frontier_threshold"))
        .def("getFrontierThreshold", &og::TRRT::getFrontierThreshold)
        .def("setFrontierNodeRatio", &og::TRRT::setFrontierNodeRatio, nb::arg("frontierNodeRatio"))
        .def("getFrontierNodeRatio", &og::TRRT::getFrontierNodeRatio);
}
