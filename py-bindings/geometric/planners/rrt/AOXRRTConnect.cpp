#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/AOXRRTConnect.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_AOXRRTConnect(nb::module_ &m)
{
    nb::class_<og::AOXRRTConnect, ob::Planner>(m, "AOXRRTConnect")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::AOXRRTConnect &self, nb::object what)
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
            "getPlannerData", [](const og::AOXRRTConnect &self, ob::PlannerData &data)
            { static_cast<const ob::Planner &>(self).getPlannerData(data); }, nb::arg("data"))
        .def("clear", [](og::AOXRRTConnect &self) { self.clear(); })
        .def("setup", &og::AOXRRTConnect::setup)
        .def("setFoundPath", &og::AOXRRTConnect::setFoundPath, nb::arg("p"))
        .def("getFoundPath", &og::AOXRRTConnect::getFoundPath)
        .def("setRange", &og::AOXRRTConnect::setRange, nb::arg("distance"))
        .def("getRange", &og::AOXRRTConnect::getRange)
        .def("internalResetCondition", &og::AOXRRTConnect::internalResetCondition)
        .def("reset", &og::AOXRRTConnect::reset, nb::arg("solvedProblem"))
        .def("setPathCost", &og::AOXRRTConnect::setPathCost, nb::arg("pc"));
}
