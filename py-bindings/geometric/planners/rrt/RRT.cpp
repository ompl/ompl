#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/OptimizationObjective.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRT(nb::module_ &m)
{
    // TAG [og::RRT][Planner]
    nb::class_<og::RRT, ob::Planner>(m, "RRT")
        // Constructor
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("addIntermediateStates") = false)

        // The destructor is virtual, automatically handled.

        // solve
        //    .def("solve",
        //         [](og::RRT &self, const ob::PlannerTerminationCondition &ptc) {
        //             return self.solve(ptc);
        //         },
        //         nb::arg("ptc"))
        .def("solve", 
               [](og::RRT &self, nb::object what){
                       if (nb::isinstance<ob::PlannerTerminationCondition>(what)) {
                            return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                       } else if (nb::isinstance<double>(what)) {
                            return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                       } else {
                            throw nb::type_error("Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                       }
               })

        // getPlannerData
        .def(
            "getPlannerData", [](const og::RRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))

        // clear
        .def("clear", &og::RRT::clear)

        // setGoalBias / getGoalBias
        .def("setGoalBias", &og::RRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::RRT::getGoalBias)

        // setIntermediateStates / getIntermediateStates
        .def("setIntermediateStates", &og::RRT::setIntermediateStates, nb::arg("addIntermediateStates"))

        .def("getIntermediateStates", &og::RRT::getIntermediateStates)

        // setRange / getRange
        .def("setRange", &og::RRT::setRange, nb::arg("distance"))
        .def("getRange", &og::RRT::getRange)
        // setup
        .def("setup", &og::RRT::setup);
}