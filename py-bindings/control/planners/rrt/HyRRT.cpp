#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/rrt/HyRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersRrt_HyRRT(nb::module_ &m)
{
    nb::class_<oc::HyRRT, ob::Planner>(m, "HyRRT")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](oc::HyRRT &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 if (nb::isinstance<double>(what))
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 throw nb::type_error(
                     "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
             })
        .def(
            "getPlannerData", [](const oc::HyRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &oc::HyRRT::clear)
        .def("setup", &oc::HyRRT::setup)
        .def("setTm", &oc::HyRRT::setTm, nb::arg("tM"))
        .def("setFlowStepDuration", &oc::HyRRT::setFlowStepDuration, nb::arg("duration"));
}
