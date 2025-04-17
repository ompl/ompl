#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/Goal.h"
#include "ompl/base/Planner.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/ScopedState.h"
#include "init.hh"

// Scoped State
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"

#include "../base/spaces/common.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_SimpleSetup(nb::module_ &m)
{
nb::class_<og::SimpleSetup>(m, "SimpleSetup")
     // Constructors
     .def(nb::init<const ob::SpaceInformationPtr &>(),
          nb::arg("si"),
          "Construct a SimpleSetup instance with a given SpaceInformation.")
     .def(nb::init<const ob::StateSpacePtr &>(),
          nb::arg("space"),
          "Construct a SimpleSetup instance with a given StateSpace.")
     
     // getSpaceInformation
     .def("getSpaceInformation",
          &og::SimpleSetup::getSpaceInformation,
          nb::rv_policy::reference_internal,
          "Return the SpaceInformation associated with this setup.")
     
     // getProblemDefinition (two overloads: const and non-const)
     .def("getProblemDefinition",
          static_cast<ob::ProblemDefinitionPtr &(og::SimpleSetup::*)()>(&og::SimpleSetup::getProblemDefinition),
          nb::rv_policy::reference_internal,
          "Return the problem definition (non-const).")
     .def("getProblemDefinitionConst",
          static_cast<const ob::ProblemDefinitionPtr &(og::SimpleSetup::*)() const>(&og::SimpleSetup::getProblemDefinition),
          nb::rv_policy::reference_internal,
          "Return the problem definition (const).")
     
     // getStateSpace
     .def("getStateSpace",
          &og::SimpleSetup::getStateSpace,
          nb::rv_policy::reference_internal,
          "Return the underlying StateSpace.")
     
     // getStateValidityChecker
     .def("getStateValidityChecker",
          &og::SimpleSetup::getStateValidityChecker,
          nb::rv_policy::reference_internal,
          "Return the current StateValidityChecker.")
     
     // getGoal
     .def("getGoal",
          &og::SimpleSetup::getGoal,
          nb::rv_policy::reference_internal,
          "Return the current Goal.")
     
     // getPlanner
     .def("getPlanner",
          &og::SimpleSetup::getPlanner,
          nb::rv_policy::reference_internal,
          "Return the planner instance used by this setup.")
     
     // getPlannerAllocator
     .def("getPlannerAllocator",
          &og::SimpleSetup::getPlannerAllocator,
          nb::rv_policy::reference_internal,
          "Return the planner allocator, if any, used by this setup.")
     
     // getPathSimplifier
     .def("getPathSimplifier",
          static_cast<og::PathSimplifierPtr &(og::SimpleSetup::*)()>(&og::SimpleSetup::getPathSimplifier),
          nb::rv_policy::reference_internal,
          "Return the PathSimplifier used by this setup (non-const).")
     .def("getPathSimplifierConst",
          static_cast<const og::PathSimplifierPtr &(og::SimpleSetup::*)() const>(&og::SimpleSetup::getPathSimplifier),
          nb::rv_policy::reference_internal,
          "Return the PathSimplifier used by this setup (const).")
     
     // getOptimizationObjective
     .def("getOptimizationObjective",
          &og::SimpleSetup::getOptimizationObjective,
          nb::rv_policy::reference_internal,
          "Return the optimization objective for this setup.")
     
     // haveExactSolutionPath, haveSolutionPath
     .def("haveExactSolutionPath",
          &og::SimpleSetup::haveExactSolutionPath,
          "Return true if the problem has an exact solution.")
     .def("haveSolutionPath",
          &og::SimpleSetup::haveSolutionPath,
          "Return true if the problem has any solution (approx or exact).")
     
     // getSolutionPlannerName
     .def("getSolutionPlannerName",
          &og::SimpleSetup::getSolutionPlannerName,
          "Return the planner name that produced the solution, if any.")
     
     // getSolutionPath returns a PathGeometric&. We should return a reference.
     .def("getSolutionPath",
          [](og::SimpleSetup &self) -> og::PathGeometric & {
               return self.getSolutionPath();
          },
          nb::rv_policy::reference_internal,
          "Return the solution path, if any (PathGeometric).")
     
     // getPlannerData
     .def("getPlannerData",
          [](og::SimpleSetup &self, ob::PlannerData &pd) {
               self.getPlannerData(pd);
          },
          nb::arg("plannerData"),
          "Fill the PlannerData with info about the current exploration.")
     
     // setStateValidityChecker (two overloads: pointer vs function).
     .def("setStateValidityChecker",
          static_cast<void (og::SimpleSetup::*)(const ob::StateValidityCheckerPtr &)>(&og::SimpleSetup::setStateValidityChecker),
          nb::arg("svc"),
          "Set the state validity checker by pointer.")
     .def("setStateValidityChecker",
          static_cast<void (og::SimpleSetup::*)(const ob::StateValidityCheckerFn &)>(&og::SimpleSetup::setStateValidityChecker),
          nb::arg("svc"),
          "Set the state validity checker by function/lambda.")
     
     // setOptimizationObjective
     .def("setOptimizationObjective",
          &og::SimpleSetup::setOptimizationObjective,
          nb::arg("objective"),
          "Set the optimization objective for planning.")
     
     // setStartAndGoalStates, addStartState, etc.
     .def("setStartAndGoalStates",
          [](og::SimpleSetup &ss, const ob::State *start, const ob::State *goal, double threshold) {
               // Retrieve the state space from the SimpleSetup.
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, start);
               auto g = state2ScopedState(space, goal);
               ss.setStartAndGoalStates(s, g, threshold);
          },
          nb::arg("start"), nb::arg("goal"),
          nb::arg("threshold") = std::numeric_limits<double>::epsilon(),
          "Set start and goal states for the problem.")
     
     .def("clearStartStates",
          &og::SimpleSetup::clearStartStates,
          "Clear all previously set start states.")

     .def("addStartState",
          [](og::SimpleSetup &ss, const ob::State * state) {
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, state);
               ss.addStartState(s);
          },
          nb::arg("state"),
          "Add an additional start state.")

     .def("setStartState",
          [](og::SimpleSetup &ss, const ob::State * state) {
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, state);
               ss.setStartState(s);
          },
          nb::arg("state"),
          "Clear existing start states and set one new start state.")

     .def("setGoalState",
          [](og::SimpleSetup &ss, const ob::State * state, double threshold) {
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, state);
               ss.setGoalState(s);
          },
          nb::arg("goal"),
          nb::arg("threshold") = std::numeric_limits<double>::epsilon(),
          "Set the goal from a single state with threshold.")        
     
     .def ("setGoal", &og::SimpleSetup::setGoal,
          nb::arg("goal"),
          "Set the goal from a Goal object.")
     .def("setPlanner",
          &og::SimpleSetup::setPlanner,
          nb::arg("planner"),
          "Set a planner for this setup.")
     .def("setPlannerAllocator",
          &og::SimpleSetup::setPlannerAllocator,
          nb::arg("allocator"),
          "Set a planner allocator function. This resets the current planner.")
     
     // solve (two overloads)
     .def("solve",
          static_cast<ob::PlannerStatus (og::SimpleSetup::*)(double)>(&og::SimpleSetup::solve),
          nb::arg("time") = 1.0,
          "Solve the problem within the given time limit.")
     .def("solve",
          static_cast<ob::PlannerStatus (og::SimpleSetup::*)(const ob::PlannerTerminationCondition &)>(&og::SimpleSetup::solve),
          nb::arg("ptc"),
          "Solve the problem until the termination condition becomes true.")
     
     // getLastPlannerStatus
     .def("getLastPlannerStatus",
          &og::SimpleSetup::getLastPlannerStatus,
          "Return the PlannerStatus from the last solve call.")
     
     // getLastPlanComputationTime, getLastSimplificationTime
     .def("getLastPlanComputationTime",
          &og::SimpleSetup::getLastPlanComputationTime,
          "Return time (seconds) spent on the last plan computation.")
     .def("getLastSimplificationTime",
          &og::SimpleSetup::getLastSimplificationTime,
          "Return time (seconds) spent on the last path simplification.")
     
     // simplifySolution (two overloads)
     .def("simplifySolution",
          static_cast<void (og::SimpleSetup::*)(double)>(&og::SimpleSetup::simplifySolution),
          nb::arg("duration") = 0.0,
          "Simplify the solution path up to a specified time duration.")
     .def("simplifySolution",
          static_cast<void (og::SimpleSetup::*)(const ob::PlannerTerminationCondition &)>(&og::SimpleSetup::simplifySolution),
          nb::arg("ptc"),
          "Simplify the solution path until the termination condition becomes true.")
     
     // clear
     .def("clear",
          &og::SimpleSetup::clear,
          "Clear the setup (including states, planner data, etc.).")
     
     // print
     .def("print",
          [](const og::SimpleSetup &self) {
               std::ostringstream oss;
               self.print(oss);
               return oss.str();
          },
          "Return a string describing the SimpleSetup.")
     
     // setup
     .def("setup",
          &og::SimpleSetup::setup,
          "Perform final configuration steps before calling solve.");
}