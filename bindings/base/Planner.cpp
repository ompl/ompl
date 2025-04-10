#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/Planner.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_Planner(nb::module_& m)
{
     nb::class_<ompl::base::Planner>(m, "Planner");

     nb::class_<ompl::base::PlannerInputStates>(m, "PlannerInputStates",
        "Wrapper class to maintain the set of input states that a planner can use")
        // Constructors
        .def(nb::init<const ompl::base::PlannerPtr&>(),
             "Constructor that takes a planner instance")
        .def(nb::init<const ompl::base::Planner*>(),
             "Constructor that takes a planner pointer")
        .def(nb::init<>(),
             "Default constructor")
        
        // Core functionality
        .def("clear", &ompl::base::PlannerInputStates::clear,
             "Clear all input states")
        .def("restart", &ompl::base::PlannerInputStates::restart,
             "Start the iteration over input states from the beginning")
        .def("update", &ompl::base::PlannerInputStates::update,
             "Update the set of input states")
        .def("use", &ompl::base::PlannerInputStates::use,
             "Use the states from the specified problem definition")
        .def("checkValidity", &ompl::base::PlannerInputStates::checkValidity,
             "Check if the input states are valid")
        
        // State iteration
        .def("nextStart", &ompl::base::PlannerInputStates::nextStart,
             "Get the next start state")
        .def("nextGoal", nb::overload_cast<>(&ompl::base::PlannerInputStates::nextGoal),
             "Get the next goal state")
        .def("nextGoal", nb::overload_cast<const ompl::base::PlannerTerminationCondition&>(&ompl::base::PlannerInputStates::nextGoal),
             "Get the next goal state with termination condition")
        
        // State availability checks
        .def("haveMoreStartStates", &ompl::base::PlannerInputStates::haveMoreStartStates,
             "Check if there are more start states")
        .def("haveMoreGoalStates", &ompl::base::PlannerInputStates::haveMoreGoalStates,
             "Check if there are more goal states")
        
        // State counting
        .def("getSeenStartStatesCount", &ompl::base::PlannerInputStates::getSeenStartStatesCount,
             "Get the number of start states seen so far")
        .def("getSampledGoalsCount", &ompl::base::PlannerInputStates::getSampledGoalsCount,
             "Get the number of sampled goal states");
}
