#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_LBTRRT(nb::module_ &m)
{
    nb::class_<og::LBTRRT, ob::Planner>(m, "LBTRRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::LBTRRT &self, nb::object what)
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
            "getPlannerData", [](const og::LBTRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LBTRRT::clear)
        .def("setup", &og::LBTRRT::setup)
        .def("setGoalBias", &og::LBTRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::LBTRRT::getGoalBias)
        .def("setRange", &og::LBTRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::LBTRRT::getRange)
        .def("setApproximationFactor", &og::LBTRRT::setApproximationFactor, nb::arg("epsilon"))
        .def("getApproximationFactor", &og::LBTRRT::getApproximationFactor);
}
