// Planner binding skeleton — copy and adapt for new geometric/control planners.
// Reference: py-bindings/geometric/planners/rrt/RRT.cpp

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRT(nb::module_ &m)
{
    nb::class_<og::RRT, ob::Planner>(m, "RRT")
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("addIntermediateStates") = false)
        .def(
            "solve",
            [](og::RRT &self, nb::object what)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                    return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                if (nb::isinstance<double>(what))
                    return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                throw nb::type_error(
                    "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
            })
        .def(
            "getPlannerData", [](const og::RRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::RRT::clear)
        .def("setup", &og::RRT::setup);
        // Add planner-specific setters/getters here.
}
