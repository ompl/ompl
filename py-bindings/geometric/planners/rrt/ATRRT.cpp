#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/ATRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_ATRRT(nb::module_ &m)
{
    nb::class_<og::ATRRT, ob::Planner>(m, "ATRRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::ATRRT &self, nb::object what)
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
            "getPlannerData", [](const og::ATRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::ATRRT::clear)
        .def("setup", &og::ATRRT::setup)
        .def("setGoalBias", &og::ATRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::ATRRT::getGoalBias)
        .def("setRange", &og::ATRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::ATRRT::getRange)
        .def("setTempChangeFactor", &og::ATRRT::setTempChangeFactor, nb::arg("factor"))
        .def("getTempChangeFactor", &og::ATRRT::getTempChangeFactor)
        .def("setCostThreshold", &og::ATRRT::setCostThreshold, nb::arg("maxCost"))
        .def("getCostThreshold", &og::ATRRT::getCostThreshold)
        .def("setInitTemperature", &og::ATRRT::setInitTemperature, nb::arg("initTemperature"))
        .def("getInitTemperature", &og::ATRRT::getInitTemperature)
        .def("setFrontierThreshold", &og::ATRRT::setFrontierThreshold, nb::arg("frontier_threshold"))
        .def("getFrontierThreshold", &og::ATRRT::getFrontierThreshold)
        .def("setFrontierNodeRatio", &og::ATRRT::setFrontierNodeRatio, nb::arg("frontierNodeRatio"))
        .def("getFrontierNodeRatio", &og::ATRRT::getFrontierNodeRatio);
}
