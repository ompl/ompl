#include <nanobind/nanobind.h>
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/ScopedState.h"

namespace nb = nanobind;

void initSimpleSetup(nb::module_& m) {
    nb::class_<ompl::geometric::SimpleSetup>(m, "SimpleSetup")
        .def(nb::init<const ompl::base::SpaceInformationPtr&>())
        .def(nb::init<const ompl::base::StateSpacePtr&>())
        
        // Space and Information getters
        .def("GetSpaceInformation", &ompl::geometric::SimpleSetup::getSpaceInformation)
        .def("GetStateSpace", &ompl::geometric::SimpleSetup::getStateSpace)
        .def("GetProblemDefinition", nb::overload_cast<>(&ompl::geometric::SimpleSetup::getProblemDefinition, nb::const_))
        
        // State and Goal Management
        .def("SetStartAndGoalStates", &ompl::geometric::SimpleSetup::setStartAndGoalStates)
        .def("SetStartState", &ompl::geometric::SimpleSetup::setStartState)
        .def("AddStartState", &ompl::geometric::SimpleSetup::addStartState)
        .def("SetGoalState", &ompl::geometric::SimpleSetup::setGoalState)
        .def("SetGoal", &ompl::geometric::SimpleSetup::setGoal)
        
        // Planning and Solution
        .def("Solve", nb::overload_cast<double>(&ompl::geometric::SimpleSetup::solve))
        .def("HaveSolutionPath", &ompl::geometric::SimpleSetup::haveSolutionPath)
        .def("GetSolutionPath", &ompl::geometric::SimpleSetup::getSolutionPath)
        .def("SimplifySolution", nb::overload_cast<double>(&ompl::geometric::SimpleSetup::simplifySolution))
        
        // Status and Timing
        .def("GetLastPlannerStatus", &ompl::geometric::SimpleSetup::getLastPlannerStatus)
        .def("GetLastPlanComputationTime", &ompl::geometric::SimpleSetup::getLastPlanComputationTime)
        .def("GetLastSimplificationTime", &ompl::geometric::SimpleSetup::getLastSimplificationTime)
        
        // Setup and Clear
        .def("Setup", &ompl::geometric::SimpleSetup::setup)
        .def("Clear", &ompl::geometric::SimpleSetup::clear);
}