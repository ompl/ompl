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
#include "init.hh"

// Scoped State
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;


template <typename Callable>
void doScopedState(og::SimpleSetup &ss,
                          const nb::object &stateObj,
                          double threshold,
                          Callable callOnTyped)
{
    auto space = ss.getSpaceInformation()->getStateSpace();
    switch (space->getType())
    {
    case ob::STATE_SPACE_REAL_VECTOR:
    {
        auto s = nb::cast<const ob::ScopedState<ob::RealVectorStateSpace> &>(stateObj);
        callOnTyped(ss, s, threshold);
        break;
    }
    case ob::STATE_SPACE_SO2:
    {
        auto s = nb::cast<const ob::ScopedState<ob::SO2StateSpace> &>(stateObj);
        callOnTyped(ss, s, threshold);
        break;
    }
    case ob::STATE_SPACE_SO3:
    {
        auto s = nb::cast<const ob::ScopedState<ob::SO3StateSpace> &>(stateObj);
        callOnTyped(ss, s, threshold);
        break;
    }
    case ob::STATE_SPACE_SE2:
    {
        auto s = nb::cast<const ob::ScopedState<ob::SE2StateSpace> &>(stateObj);
        callOnTyped(ss, s, threshold);
        break;
    }
    case ob::STATE_SPACE_SE3:
    {
        auto s = nb::cast<const ob::ScopedState<ob::SE3StateSpace> &>(stateObj);
        callOnTyped(ss, s, threshold);
        break;
    }
   
    // case ob::STATE_SPACE_TIME:
    // {
    //     auto s = nb::cast<const ob::ScopedState<ob::TimeStateSpace>&>(stateObj);
    //     callOnTyped(ss, s, threshold);
    //     break;
    // }
    //

    default:
        throw std::runtime_error("Unsupported or unhandled state space type.");
    }
}

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
        // TODO: Template
        .def("setStartAndGoalStates",
            [](og::SimpleSetup &ss, const nb::object &start, const nb::object &goal, double threshold) {
                // Retrieve the state space from the SimpleSetup.
                auto space = ss.getSpaceInformation()->getStateSpace();
                switch (space->getType())
                {
                case ob::STATE_SPACE_REAL_VECTOR:
                {
                    auto s = nb::cast<const ob::ScopedState<ob::RealVectorStateSpace>&>(start);
                    auto g = nb::cast<const ob::ScopedState<ob::RealVectorStateSpace>&>(goal);
                    ss.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ob::STATE_SPACE_SO2:
                {
                    auto s = nb::cast<const ob::ScopedState<ob::SO2StateSpace>&>(start);
                    auto g = nb::cast<const ob::ScopedState<ob::SO2StateSpace>&>(goal);
                    ss.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ob::STATE_SPACE_SO3:
                {
                    auto s = nb::cast<const ob::ScopedState<ob::SO3StateSpace>&>(start);
                    auto g = nb::cast<const ob::ScopedState<ob::SO3StateSpace>&>(goal);
                    ss.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ob::STATE_SPACE_SE2:
                {
                    auto s = nb::cast<const ob::ScopedState<ob::SE2StateSpace>&>(start);
                    auto g = nb::cast<const ob::ScopedState<ob::SE2StateSpace>&>(goal);
                    ss.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ob::STATE_SPACE_SE3:
                {
                    auto s = nb::cast<const ob::ScopedState<ob::SE3StateSpace>&>(start);
                    auto g = nb::cast<const ob::ScopedState<ob::SE3StateSpace>&>(goal);
                    ss.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                default:
                    throw std::runtime_error("Unsupported or unhandled state space type.");
                }
            },
            nb::arg("start"), nb::arg("goal"),
            nb::arg("threshold") = std::numeric_limits<double>::epsilon(),
            "Set start and goal states for the problem (dynamic-dispatch version).")
        
        .def("clearStartStates",
            &og::SimpleSetup::clearStartStates,
            "Clear all previously set start states.")

            .def("addStartState",
                [](og::SimpleSetup &ss, const nb::object &stateObj) {
                    doScopedState(ss, stateObj, 0.0, [](og::SimpleSetup &mys,
                                                               auto &typedState,
                                                               double /*thr*/) {
                        mys.addStartState(typedState);
                    });
                },
                nb::arg("state"),
                "Add an additional start state (dynamic-dispatch version).")

        .def("setStartState",
            [](og::SimpleSetup &ss, const nb::object &stateObj) {
                doScopedState(ss, stateObj, 0.0, [](og::SimpleSetup &mys,
                                                            auto &typedState,
                                                            double /*thr*/) {
                    mys.setStartState(typedState);
                });
            },
            nb::arg("state"),
            "Clear existing start states and set one new start state (dynamic-dispatch).")

        .def("setGoalState",
            [](og::SimpleSetup &ss, const nb::object &goalObj, double threshold) {
                doScopedState(ss, goalObj, threshold, [](og::SimpleSetup &mys,
                                                                auto &typedGoal,
                                                                double thr) {
                    mys.setGoalState(typedGoal, thr);
                });
            },
            nb::arg("goal"),
            nb::arg("threshold") = std::numeric_limits<double>::epsilon(),
            "Set the goal from a single state with threshold (dynamic-dispatch).")        
        
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