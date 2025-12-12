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
     // TAG [oc::RRT][Planner]
    nb::class_<oc::RRT, ob::Planner>(m, "RRT")
        // --- Constructor
        .def(nb::init<const oc::SpaceInformationPtr &>(),
             nb::arg("si"))

        // --- Overridden methods from base::Planner
        .def("solve",
             &oc::RRT::solve,
             nb::arg("terminationCondition"))
        .def("clear", &oc::RRT::clear)
        
        // --- RRT-specific methods
        .def("setGoalBias", &oc::RRT::setGoalBias,
             nb::arg("goalBias"))
        .def("getGoalBias", &oc::RRT::getGoalBias)

        .def("setIntermediateStates", &oc::RRT::setIntermediateStates,
             nb::arg("addIntermediateStates"))
        .def("getIntermediateStates", &oc::RRT::getIntermediateStates)

        // getPlannerData (fills the PlannerData object)
        .def("getPlannerData",
             [](const oc::RRT &rrt, ob::PlannerData &data) {
                 rrt.getPlannerData(data);
             },
             nb::arg("data"))
        // set up
        .def("setup", &oc::RRT::setup);
}
