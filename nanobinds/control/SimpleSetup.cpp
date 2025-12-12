#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/control/SimpleSetup.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/PathControl.h"
#include "ompl/base/Planner.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/OptimizationObjective.h"
#include "../base/spaces/common.hh"

#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_SimpleSetup(nb::module_ &m)
{
    nb::class_<oc::SimpleSetup>(m, "SimpleSetup")
        // Constructors
        .def(nb::init<oc::SpaceInformationPtr>(), nb::arg("spaceInfo"))
        .def(nb::init<oc::ControlSpacePtr>(), nb::arg("controlSpace"))

        // getSpaceInformation, returns SpaceInformationPtr
        .def("getSpaceInformation", &oc::SimpleSetup::getSpaceInformation, nb::rv_policy::reference_internal)

        // ProblemDefinition
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)()>(&oc::SimpleSetup::getProblemDefinition))
        .def("getProblemDefinitionConst",
             static_cast<const ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)() const>(
                 &oc::SimpleSetup::getProblemDefinition))

        // getStateSpace, getControlSpace
        .def("getStateSpace", &oc::SimpleSetup::getStateSpace, nb::rv_policy::reference_internal)
        .def("getControlSpace", &oc::SimpleSetup::getControlSpace, nb::rv_policy::reference_internal)

        // Common getters
        .def("getStateValidityChecker", &oc::SimpleSetup::getStateValidityChecker, nb::rv_policy::reference_internal)
        .def("getStatePropagator", &oc::SimpleSetup::getStatePropagator, nb::rv_policy::reference_internal)
        .def("getGoal", &oc::SimpleSetup::getGoal, nb::rv_policy::reference_internal)
        .def("getPlanner", &oc::SimpleSetup::getPlanner, nb::rv_policy::reference_internal)
        .def("getPlannerAllocator", &oc::SimpleSetup::getPlannerAllocator, nb::rv_policy::reference_internal)

        // solution path checks
        .def("haveExactSolutionPath", &oc::SimpleSetup::haveExactSolutionPath)
        .def("haveSolutionPath", &oc::SimpleSetup::haveSolutionPath)

        // getSolutionPath: returns a PathControl& => use reference_internal
        .def(
            "getSolutionPath", [](oc::SimpleSetup &self) -> oc::PathControl & { return self.getSolutionPath(); },
            nb::rv_policy::reference_internal)

        // getPlannerData
        .def(
            "getPlannerData", [](const oc::SimpleSetup &ss, ob::PlannerData &pd) { ss.getPlannerData(pd); },
            nb::arg("plannerData"), nb::rv_policy::reference_internal)

        // setStateValidityChecker (two overloads)
        .def("setStateValidityChecker", nb::overload_cast<const ompl::base::StateValidityCheckerFn &>(
                                            &ompl::control::SimpleSetup::setStateValidityChecker))
        // setStatePropagator
        .def(
            "setStatePropagator",
            static_cast<void (oc::SimpleSetup::*)(const oc::StatePropagatorFn &)>(&oc::SimpleSetup::setStatePropagator))
        .def("setStatePropagator",
             static_cast<void (oc::SimpleSetup::*)(const oc::StatePropagatorPtr &)>(
                 &oc::SimpleSetup::setStatePropagator))

        // setOptimizationObjective
        .def("setOptimizationObjective", &oc::SimpleSetup::setOptimizationObjective, nb::arg("objective"))

        // start/goal states
        .def(
            "setStartAndGoalStates",
            [](oc::SimpleSetup &ss, const ob::State *start, const ob::State *goal, double threshold)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto s = state2ScopedState(space, start);
                auto g = state2ScopedState(space, goal);
                ss.setStartAndGoalStates(s, g, threshold);
            },
            nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def(
            "setGoalState",
            [](oc::SimpleSetup &ss, const ob::State *goal, double threshold)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto g = state2ScopedState(space, goal);
                ss.setGoalState(g, threshold);
            },
            nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def(
            "addStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto s = state2ScopedState(space, state);
                ss.addStartState(s);
            },
            nb::arg("state"))
        .def("clearStartStates", &oc::SimpleSetup::clearStartStates)
        .def(
            "setStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto s = state2ScopedState(space, state);
                ss.setStartState(s);
            },
            nb::arg("state"))
        .def("setGoal", &oc::SimpleSetup::setGoal, nb::arg("goal"))

        // setPlanner, setPlannerAllocator
        .def("setPlanner", &oc::SimpleSetup::setPlanner, nb::arg("planner"))
        .def("setPlannerAllocator", &oc::SimpleSetup::setPlannerAllocator, nb::arg("allocator"))

        // solve() methods (two overloads)
        .def("solve", nb::overload_cast<double>(&oc::SimpleSetup::solve), nb::arg("time") = 1.0)
        .def("solve", nb::overload_cast<const ob::PlannerTerminationCondition &>(&oc::SimpleSetup::solve),
             nb::arg("terminationCondition"))

        // getLastPlannerStatus, getLastPlanComputationTime
        .def("getLastPlannerStatus", &oc::SimpleSetup::getLastPlannerStatus)
        .def("getLastPlanComputationTime", &oc::SimpleSetup::getLastPlanComputationTime)

        // clear
        .def("clear", &oc::SimpleSetup::clear)

        // print
        .def(
            "print",
            [](const oc::SimpleSetup &ss)
            {
                std::ostringstream oss;
                ss.print(oss);
                return oss.str();
            })

        // setup
        .def("setup", &oc::SimpleSetup::setup);
}
