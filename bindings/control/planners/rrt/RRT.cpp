#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/PlannerData.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersRrt_RRT(nb::module_ &m)
{
    nb::class_<oc::RRT, ob::Planner>(m, "RRT")
        // --- Constructor
        .def(nb::init<const oc::SpaceInformationPtr >(),
             nb::arg("si"),
             "Create an RRT instance for a control-based SpaceInformation.")

        // --- Overridden methods from base::Planner
        .def("solve",
             &oc::RRT::solve,
             nb::arg("terminationCondition"),
             "Attempt to solve the planning problem until the specified termination condition becomes true. "
             "Return the PlannerStatus.")
        .def("clear", &oc::RRT::clear,
             "Clear all states and datastructures used by the planner.")
        
        // --- RRT-specific methods
        .def("setGoalBias", &oc::RRT::setGoalBias,
             nb::arg("goalBias"),
             "Set the fraction of random selections that target the goal (between 0 and 1).")
        .def("getGoalBias", &oc::RRT::getGoalBias,
             "Return the current goal bias fraction.")

        .def("setIntermediateStates", &oc::RRT::setIntermediateStates,
             nb::arg("addIntermediateStates"),
             "Set whether intermediate states along a motion are added to the tree.")
        .def("getIntermediateStates", &oc::RRT::getIntermediateStates,
             "Return whether intermediate states are being added to the tree.")

        // getPlannerData (fills the PlannerData object)
        .def("getPlannerData",
             [](const oc::RRT &rrt, ob::PlannerData &data) {
                 rrt.getPlannerData(data);
             },
             nb::arg("data"),
             "Fill PlannerData with information about the exploration data structure used by RRT.")
        // set up
        .def("setup", &oc::RRT::setup,
             "Do any final setup steps for RRT (e.g. verifying settings).");
}
