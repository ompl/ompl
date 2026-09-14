#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/LazyLBTRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_LazyLBTRRT(nb::module_ &m)
{
    nb::class_<og::LazyLBTRRT, ob::Planner>(m, "LazyLBTRRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::LazyLBTRRT &self, nb::object what)
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
            "getPlannerData", [](const og::LazyLBTRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LazyLBTRRT::clear)
        .def("setup", &og::LazyLBTRRT::setup)
        .def("setGoalBias", &og::LazyLBTRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::LazyLBTRRT::getGoalBias)
        .def("setRange", &og::LazyLBTRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::LazyLBTRRT::getRange)
        .def("setApproximationFactor", &og::LazyLBTRRT::setApproximationFactor, nb::arg("epsilon"));
}
