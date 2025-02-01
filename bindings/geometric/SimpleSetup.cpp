#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

#include "ompl/base/State.h"
#include "ompl/base/ScopedState.h"

namespace nb = nanobind;

void initSimpleSetup(nb::module_& m) {    
    nb::class_<ompl::geometric::SimpleSetup>(m, "SimpleSetup")
        .def(nb::init<const ompl::base::SpaceInformationPtr&>())
        .def(nb::init<const ompl::base::StateSpacePtr&>())
        
        // Space and Information getters
        .def("getSpaceInformation", &ompl::geometric::SimpleSetup::getSpaceInformation)
        .def("getStateSpace", &ompl::geometric::SimpleSetup::getStateSpace)
        .def("getProblemDefinition", nb::overload_cast<>(&ompl::geometric::SimpleSetup::getProblemDefinition, nb::const_))
        
        // State and Goal Management
        .def("setStartAndGoalStates", [](ompl::geometric::SimpleSetup& self, 
                                        const ompl::base::ScopedState<ompl::base::SE2StateSpace>& start, 
                                        const ompl::base::ScopedState<ompl::base::SE2StateSpace>& goal, 
                                        double threshold = std::numeric_limits<double>::epsilon()) {
            self.setStartAndGoalStates(start, goal, threshold);
        }, nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())

        .def("setStartState", &ompl::geometric::SimpleSetup::setStartState)
        .def("addStartState", &ompl::geometric::SimpleSetup::addStartState)
        .def("setGoalState", &ompl::geometric::SimpleSetup::setGoalState)
        .def("setGoal", &ompl::geometric::SimpleSetup::setGoal)
        .def("setStateValidityChecker", nb::overload_cast<const ompl::base::StateValidityCheckerFn&>(&ompl::geometric::SimpleSetup::setStateValidityChecker))
        // Planning and Solution
        .def("solve", nb::overload_cast<double>(&ompl::geometric::SimpleSetup::solve))
        .def("haveSolutionPath", &ompl::geometric::SimpleSetup::haveSolutionPath)
        .def("getSolutionPath", &ompl::geometric::SimpleSetup::getSolutionPath)
        .def("simplifySolution", nb::overload_cast<double>(&ompl::geometric::SimpleSetup::simplifySolution))
        
        // Status and Timing
        .def("getLastPlannerStatus", &ompl::geometric::SimpleSetup::getLastPlannerStatus)
        .def("getLastPlanComputationTime", &ompl::geometric::SimpleSetup::getLastPlanComputationTime)
        .def("getLastSimplificationTime", &ompl::geometric::SimpleSetup::getLastSimplificationTime)
        .def("getStateValidityChecker", &ompl::geometric::SimpleSetup::getStateValidityChecker)
        // Setup and Clear
        .def("setup", &ompl::geometric::SimpleSetup::setup)
        .def("clear", &ompl::geometric::SimpleSetup::clear);
}