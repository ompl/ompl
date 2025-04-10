#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/OptimizationObjective.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRT(nb::module_ &m)
{
    nb::class_<og::RRT, ob::Planner>(m, "RRT")
        // Constructor
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(),
             nb::arg("si"), nb::arg("addIntermediateStates") = false,
             "Construct an RRT instance with an optional flag for intermediate states.")
        
        // The destructor is virtual, automatically handled.
        
        // solve
        .def("solve",
             [](og::RRT &self, const ob::PlannerTerminationCondition &ptc) {
                 return self.solve(ptc);
             },
             nb::arg("ptc"),
             "Try to solve the motion planning problem within the termination condition.")
        
        // getPlannerData
        .def("getPlannerData",
             [](const og::RRT &self, ob::PlannerData &data) {
                 self.getPlannerData(data);
             },
             nb::arg("data"),
             "Populate PlannerData with information about the exploration data structure used by this planner.")
        
        // clear
        .def("clear", &og::RRT::clear,
             "Clear all datastructures for this planner.")
        
        // setGoalBias / getGoalBias
        .def("setGoalBias", &og::RRT::setGoalBias,
             nb::arg("goalBias"),
             "Set the probability [0,1] of sampling the goal state (default is 0.05).")
        .def("getGoalBias", &og::RRT::getGoalBias,
             "Get the current goal bias.")
        
        // setIntermediateStates / getIntermediateStates
        .def("setIntermediateStates", &og::RRT::setIntermediateStates,
             nb::arg("addIntermediateStates"),
             "Set whether intermediate states are added to the tree during extension.")
        .def("getIntermediateStates", &og::RRT::getIntermediateStates,
             "Return true if intermediate states are added in the extension steps.")
        
        // setRange / getRange
        .def("setRange", &og::RRT::setRange,
             nb::arg("distance"),
             "Set the maximum distance between consecutive states in the tree.")
        .def("getRange", &og::RRT::getRange,
             "Return the maximum distance between consecutive states in the tree.")
        // setup
        .def("setup", &og::RRT::setup,
             "Setup the planner (call before solve).");
}