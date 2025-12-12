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
          nb::arg("si"))
     .def(nb::init<const ob::StateSpacePtr &>(),
          nb::arg("space"))
     
     // getSpaceInformation
     .def("getSpaceInformation",
          &og::SimpleSetup::getSpaceInformation,
          nb::rv_policy::reference_internal)
     
     // getProblemDefinition (two overloads: const and non-const)
     .def("getProblemDefinition",
          static_cast<ob::ProblemDefinitionPtr &(og::SimpleSetup::*)()>(&og::SimpleSetup::getProblemDefinition),
          nb::rv_policy::reference_internal)
     .def("getProblemDefinitionConst",
          static_cast<const ob::ProblemDefinitionPtr &(og::SimpleSetup::*)() const>(&og::SimpleSetup::getProblemDefinition),
          nb::rv_policy::reference_internal)
     
     // getStateSpace
     .def("getStateSpace",
          &og::SimpleSetup::getStateSpace,
          nb::rv_policy::reference_internal)
     
     // getStateValidityChecker
     .def("getStateValidityChecker",
          &og::SimpleSetup::getStateValidityChecker,
          nb::rv_policy::reference_internal)
     
     // getGoal
     .def("getGoal",
          &og::SimpleSetup::getGoal,
          nb::rv_policy::reference_internal)
     
     // getPlanner
     .def("getPlanner",
          &og::SimpleSetup::getPlanner,
          nb::rv_policy::reference_internal)
     
     // getPlannerAllocator
     .def("getPlannerAllocator",
          &og::SimpleSetup::getPlannerAllocator,
          nb::rv_policy::reference_internal)
     
     // getPathSimplifier
     .def("getPathSimplifier",
          static_cast<og::PathSimplifierPtr &(og::SimpleSetup::*)()>(&og::SimpleSetup::getPathSimplifier),
          nb::rv_policy::reference_internal)
     .def("getPathSimplifierConst",
          static_cast<const og::PathSimplifierPtr &(og::SimpleSetup::*)() const>(&og::SimpleSetup::getPathSimplifier),
          nb::rv_policy::reference_internal)
     
     // getOptimizationObjective
     .def("getOptimizationObjective",
          &og::SimpleSetup::getOptimizationObjective,
          nb::rv_policy::reference_internal)
     
     // haveExactSolutionPath, haveSolutionPath
     .def("haveExactSolutionPath",
          &og::SimpleSetup::haveExactSolutionPath)
     .def("haveSolutionPath",
          &og::SimpleSetup::haveSolutionPath)
     
     // getSolutionPlannerName
     .def("getSolutionPlannerName",
          &og::SimpleSetup::getSolutionPlannerName)
     
     // getSolutionPath returns a PathGeometric&. We should return a reference.
     .def("getSolutionPath",
          [](og::SimpleSetup &self) -> og::PathGeometric & {
               return self.getSolutionPath();
          },
          nb::rv_policy::reference_internal)
     
     // getPlannerData
     .def("getPlannerData",
          [](og::SimpleSetup &self, ob::PlannerData &pd) {
               self.getPlannerData(pd);
          },
          nb::arg("plannerData"))
     
     // setStateValidityChecker (two overloads: pointer vs function).
     .def("setStateValidityChecker",
          static_cast<void (og::SimpleSetup::*)(const ob::StateValidityCheckerPtr &)>(&og::SimpleSetup::setStateValidityChecker),
          nb::arg("svc"))
     .def("setStateValidityChecker",
          static_cast<void (og::SimpleSetup::*)(const ob::StateValidityCheckerFn &)>(&og::SimpleSetup::setStateValidityChecker),
          nb::arg("svc"))
     
     // setOptimizationObjective
     .def("setOptimizationObjective",
          &og::SimpleSetup::setOptimizationObjective,
          nb::arg("objective"))
     
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
          nb::arg("threshold") = std::numeric_limits<double>::epsilon())
     
     .def("clearStartStates",
          &og::SimpleSetup::clearStartStates)

     .def("addStartState",
          [](og::SimpleSetup &ss, const ob::State * state) {
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, state);
               ss.addStartState(s);
          },
          nb::arg("state"))

     .def("setStartState",
          [](og::SimpleSetup &ss, const ob::State * state) {
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, state);
               ss.setStartState(s);
          },
          nb::arg("state"))

     .def("setGoalState",
          [](og::SimpleSetup &ss, const ob::State * state, double threshold) {
               auto space = ss.getSpaceInformation()->getStateSpace();
               auto s = state2ScopedState(space, state);
               ss.setGoalState(s);
          },
          nb::arg("goal"),
          nb::arg("threshold") = std::numeric_limits<double>::epsilon())        
     
     .def ("setGoal", &og::SimpleSetup::setGoal,
          nb::arg("goal"))
     .def("setPlanner",
          &og::SimpleSetup::setPlanner,
          nb::arg("planner"),
          "Set a planner for this setup.")
     .def("setPlannerAllocator",
          &og::SimpleSetup::setPlannerAllocator,
          nb::arg("allocator"))
     
     // solve (two overloads)
     .def("solve",
          static_cast<ob::PlannerStatus (og::SimpleSetup::*)(double)>(&og::SimpleSetup::solve),
          nb::arg("time") = 1.0)
     .def("solve",
          static_cast<ob::PlannerStatus (og::SimpleSetup::*)(const ob::PlannerTerminationCondition &)>(&og::SimpleSetup::solve),
          nb::arg("ptc"))
     
     // getLastPlannerStatus
     .def("getLastPlannerStatus",
          &og::SimpleSetup::getLastPlannerStatus)
     
     // getLastPlanComputationTime, getLastSimplificationTime
     .def("getLastPlanComputationTime",
          &og::SimpleSetup::getLastPlanComputationTime)
     .def("getLastSimplificationTime",
          &og::SimpleSetup::getLastSimplificationTime)
     
     // simplifySolution (two overloads)
     .def("simplifySolution",
          static_cast<void (og::SimpleSetup::*)(double)>(&og::SimpleSetup::simplifySolution),
          nb::arg("duration") = 0.0)
     .def("simplifySolution",
          static_cast<void (og::SimpleSetup::*)(const ob::PlannerTerminationCondition &)>(&og::SimpleSetup::simplifySolution),
          nb::arg("ptc"))
     
     // clear
     .def("clear",
          &og::SimpleSetup::clear)
     
     // print
     .def("print",
          [](const og::SimpleSetup &self) {
               std::ostringstream oss;
               self.print(oss);
               return oss.str();
          })
     
     // setup
     .def("setup",
          &og::SimpleSetup::setup);
}