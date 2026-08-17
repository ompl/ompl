#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/planners/ltl/LTLPlanner.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersLtl_LTLPlanner(nb::module_ &m)
{
    nb::class_<oc::LTLPlanner, ob::Planner>(m, "LTLPlanner")
        .def(nb::init<const oc::LTLSpaceInformationPtr &, oc::ProductGraphPtr, double>(), nb::arg("si"), nb::arg("a"),
             nb::arg("exploreTime") = 0.5)
        .def("solve",
             [](oc::LTLPlanner &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 if (nb::isinstance<double>(what))
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 throw nb::type_error(
                     "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
             })
        .def("clear", &oc::LTLPlanner::clear)
        .def("setup", &oc::LTLPlanner::setup)
        .def("getTree", &oc::LTLPlanner::getTree, nb::arg("tree"));
}
