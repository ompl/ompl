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
        .def(nb::init<oc::SpaceInformationPtr>(), nb::arg("spaceInfo"),
             "Construct a SimpleSetup given a SpaceInformation pointer.")
        .def(nb::init<oc::ControlSpacePtr>(), nb::arg("controlSpace"),
             "Construct a SimpleSetup from a ControlSpace (internally building a new SpaceInformation).")

        // getSpaceInformation, returns SpaceInformationPtr
        .def("getSpaceInformation", &oc::SimpleSetup::getSpaceInformation, nb::rv_policy::reference_internal,
             "Return the underlying SpaceInformation (control-based).")

        // ProblemDefinition
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)()>(&oc::SimpleSetup::getProblemDefinition),
             nb::rv_policy::reference_internal, "Return the problem definition (non-const).")
        .def("getProblemDefinitionConst",
             static_cast<const ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)() const>(
                 &oc::SimpleSetup::getProblemDefinition),
             nb::rv_policy::reference_internal, "Return the problem definition (const).")

        // getStateSpace, getControlSpace
        .def("getStateSpace", &oc::SimpleSetup::getStateSpace, nb::rv_policy::reference_internal,
             "Return the underlying state space.")
        .def("getControlSpace", &oc::SimpleSetup::getControlSpace, nb::rv_policy::reference_internal,
             "Return the underlying control space.")

        // Common getters
        .def("getStateValidityChecker", &oc::SimpleSetup::getStateValidityChecker, nb::rv_policy::reference_internal,
             "Return the current state validity checker.")
        .def("getStatePropagator", &oc::SimpleSetup::getStatePropagator, nb::rv_policy::reference_internal,
             "Return the currently set state propagator.")
        .def("getGoal", &oc::SimpleSetup::getGoal, nb::rv_policy::reference_internal,
             "Return the current goal definition.")
        .def("getPlanner", &oc::SimpleSetup::getPlanner, nb::rv_policy::reference_internal,
             "Return the current planner, if any.")
        .def("getPlannerAllocator", &oc::SimpleSetup::getPlannerAllocator, nb::rv_policy::reference_internal,
             "Return the current planner allocator function, if any.")

        // solution path checks
        .def("haveExactSolutionPath", &oc::SimpleSetup::haveExactSolutionPath,
             "Return whether an exact solution path is found.")
        .def("haveSolutionPath", &oc::SimpleSetup::haveSolutionPath,
             "Return whether any solution path is found (approx or exact).")

        // getSolutionPath: returns a PathControl& => use reference_internal
        .def(
            "getSolutionPath", [](oc::SimpleSetup &self) -> oc::PathControl & { return self.getSolutionPath(); },
            nb::rv_policy::reference_internal,
            "Return the current solution path as a PathControl object.")

        // getPlannerData
        .def(
            "getPlannerData", [](const oc::SimpleSetup &ss, ob::PlannerData &pd) { ss.getPlannerData(pd); },
            nb::arg("plannerData"), nb::rv_policy::reference_internal,
            "Fill the provided PlannerData structure with the exploration data from the current planner.")

        // setStateValidityChecker (two overloads)
        .def("setStateValidityChecker", nb::overload_cast<const ompl::base::StateValidityCheckerFn &>(
                                            &ompl::control::SimpleSetup::setStateValidityChecker))
        // setStatePropagator
        .def(
            "setStatePropagator",
            static_cast<void (oc::SimpleSetup::*)(const oc::StatePropagatorFn &)>(&oc::SimpleSetup::setStatePropagator),
            nb::arg("propagatorFn"), "Set the state propagator from a function/lambda.")
        .def("setStatePropagator",
             static_cast<void (oc::SimpleSetup::*)(const oc::StatePropagatorPtr &)>(
                 &oc::SimpleSetup::setStatePropagator),
             nb::arg("propagatorPtr"), "Set the state propagator from a StatePropagator pointer.")

        // setOptimizationObjective
        .def("setOptimizationObjective", &oc::SimpleSetup::setOptimizationObjective, nb::arg("objective"),
             "Set the optimization objective for planning, if desired.")

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
            nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon(),
            "Set a single start state and a single goal state.")
        .def(
            "setGoalState",
            [](oc::SimpleSetup &ss, const ob::State *goal, double threshold)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto g = state2ScopedState(space, goal);
                ss.setGoalState(g, threshold);
            },
            nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon(),
            "Set the goal from a single state.")
        .def(
            "addStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto s = state2ScopedState(space, state);
                ss.addStartState(s);
            },
            nb::arg("state"), "Add an additional start state.")
        .def("clearStartStates", &oc::SimpleSetup::clearStartStates, "Clear all previously-set start states.")
        .def(
            "setStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                auto s = state2ScopedState(space, state);
                ss.setStartState(s);
            },
            nb::arg("state"), "Clear existing start states and set one new start state.")
        .def("setGoal", &oc::SimpleSetup::setGoal, nb::arg("goal"), "Set the goal pointer directly.")

        // setPlanner, setPlannerAllocator
        .def("setPlanner", &oc::SimpleSetup::setPlanner, nb::arg("planner"),
             "Set a planner instance for this setup. Must match the SpaceInformation.")
        .def("setPlannerAllocator", &oc::SimpleSetup::setPlannerAllocator, nb::arg("allocator"),
             "Set a function/functor that creates a Planner. Overwrites any existing planner.")

        // solve() methods (two overloads)
        .def("solve", nb::overload_cast<double>(&oc::SimpleSetup::solve), nb::arg("time") = 1.0,
             "Attempt to solve the problem within the specified time limit.")
        .def("solve", nb::overload_cast<const ob::PlannerTerminationCondition &>(&oc::SimpleSetup::solve),
             nb::arg("terminationCondition"),
             "Attempt to solve the problem until the given termination condition is satisfied.")

        // getLastPlannerStatus, getLastPlanComputationTime
        .def("getLastPlannerStatus", &oc::SimpleSetup::getLastPlannerStatus,
             "Return the PlannerStatus result from the last solve call.")
        .def("getLastPlanComputationTime", &oc::SimpleSetup::getLastPlanComputationTime,
             "Return how many seconds the last solve invocation took to find or fail to find a solution.")

        // clear
        .def("clear", &oc::SimpleSetup::clear, "Clear the setup, forgetting all states, solutions, and planners.")

        // print
        .def(
            "print",
            [](const oc::SimpleSetup &ss)
            {
                std::ostringstream oss;
                ss.print(oss);
                return oss.str();
            },
            "Return a string describing the SimpleSetup configuration.")

        // setup
        .def("setup", &oc::SimpleSetup::setup, "Finalize the setup (call before solve).");
}
