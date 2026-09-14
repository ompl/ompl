#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rlrt/RLRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRlrt_RLRT(nb::module_ &m)
{
    nb::class_<og::RLRT, ob::Planner>(m, "RLRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::RLRT &self, nb::object what)
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
            "getPlannerData", [](const og::RLRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::RLRT::clear)
        .def("setup", &og::RLRT::setup)
        .def("setGoalBias", &og::RLRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::RLRT::getGoalBias)
        .def("setRange", &og::RLRT::setRange, nb::arg("distance"))
        .def("getRange", &og::RLRT::getRange)
        .def("getKeepLast", &og::RLRT::getKeepLast)
        .def("setKeepLast", &og::RLRT::setKeepLast, nb::arg("keepLast"));
}
