#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/BiTRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_BiTRRT(nb::module_ &m)
{
    nb::class_<og::BiTRRT, ob::Planner>(m, "BiTRRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::BiTRRT &self, nb::object what)
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
            "getPlannerData", [](const og::BiTRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::BiTRRT::clear)
        .def("setup", &og::BiTRRT::setup)
        .def("setRange", &og::BiTRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::BiTRRT::getRange)
        .def("setTempChangeFactor", &og::BiTRRT::setTempChangeFactor, nb::arg("factor"))
        .def("getTempChangeFactor", &og::BiTRRT::getTempChangeFactor)
        .def("setCostThreshold", &og::BiTRRT::setCostThreshold, nb::arg("maxCost"))
        .def("getCostThreshold", &og::BiTRRT::getCostThreshold)
        .def("setInitTemperature", &og::BiTRRT::setInitTemperature, nb::arg("initTemperature"))
        .def("getInitTemperature", &og::BiTRRT::getInitTemperature)
        .def("setFrontierThreshold", &og::BiTRRT::setFrontierThreshold, nb::arg("frontierThreshold"))
        .def("getFrontierThreshold", &og::BiTRRT::getFrontierThreshold)
        .def("setFrontierNodeRatio", &og::BiTRRT::setFrontierNodeRatio, nb::arg("frontierNodeRatio"))
        .def("getFrontierNodeRatio", &og::BiTRRT::getFrontierNodeRatio);
}
