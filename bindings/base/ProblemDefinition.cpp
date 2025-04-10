#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <sstream>

// Include the OMPL headers for ProblemDefinition and related classes.
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/Goal.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SolutionNonExistenceProof.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"

#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_ProblemDefinition(nb::module_ &m) {
    nb::class_<ompl::base::ProblemDefinition>(m, "ProblemDefinition")
        // Deleted copy constructor and assignment are not bound.
        // Bind the constructor that takes a SpaceInformationPtr.
        .def(nb::init<ompl::base::SpaceInformationPtr>())

        // clone()
        .def("clone", &ompl::base::ProblemDefinition::clone)

        // getSpaceInformation() returns a reference; use reference_internal
        .def("getSpaceInformation", &ompl::base::ProblemDefinition::getSpaceInformation,
             nb::rv_policy::reference_internal)

        // addStartState overloads.
        .def("addStartState", nb::overload_cast<const ompl::base::State*>
             (&ompl::base::ProblemDefinition::addStartState))
        .def("addStartState", nb::overload_cast<const ompl::base::ScopedState<> &>
             (&ompl::base::ProblemDefinition::addStartState))

        // hasStartState: note that the function takes an optional unsigned int pointer.
        .def("hasStartState", nb::overload_cast<const ompl::base::State*, unsigned int*>
             (&ompl::base::ProblemDefinition::hasStartState, nb::const_))

        // clearStartStates
        .def("clearStartStates", &ompl::base::ProblemDefinition::clearStartStates)

        // getStartStateCount
        .def("getStartStateCount", &ompl::base::ProblemDefinition::getStartStateCount)

        // getStartState overloads.
        .def("getStartState", nb::overload_cast<unsigned int>
             (&ompl::base::ProblemDefinition::getStartState, nb::const_))
        .def("getStartState", nb::overload_cast<unsigned int>
             (&ompl::base::ProblemDefinition::getStartState))

        // setGoal and clearGoal
        .def("setGoal", &ompl::base::ProblemDefinition::setGoal)
        .def("clearGoal", &ompl::base::ProblemDefinition::clearGoal)

        // getGoal returns a reference.
        .def("getGoal", &ompl::base::ProblemDefinition::getGoal,
             nb::rv_policy::reference_internal)

        // getInputStates: Since the function fills a passed vector,
        // we wrap it so that it returns the vector to Python.
        .def("getInputStates", [](const ompl::base::ProblemDefinition &pd) {
            std::vector<const ompl::base::State*> states;
            pd.getInputStates(states);
            return states;
        })

        // setStartAndGoalStates (State* version) with default threshold.
        .def("setStartAndGoalStates", nb::overload_cast<const ompl::base::State*, const ompl::base::State*, double>
             (&ompl::base::ProblemDefinition::setStartAndGoalStates),
             nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())

        // setGoalState (State* version) with default threshold.
        .def("setGoalState", nb::overload_cast<const ompl::base::State*, double>
             (&ompl::base::ProblemDefinition::setGoalState),
             nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())

     //    setStartAndGoalStates (ScopedState<> version)
        .def("setStartAndGoalStates", nb::overload_cast<const ompl::base::ScopedState<>&,
                                                 const ompl::base::ScopedState<>&, double>
             (&ompl::base::ProblemDefinition::setStartAndGoalStates),
             nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def("setStartAndGoalStates", [](ompl::base::ProblemDefinition &pd,
            const nb::object &start,
            const nb::object &goal,
            double threshold) {
            // Retrieve the state space from the space information.
            auto space = pd.getSpaceInformation()->getStateSpace();

            // Dispatch based on the state space type.
            switch (space->getType()) {
                case ompl::base::STATE_SPACE_REAL_VECTOR: {
                    auto s = nb::cast<const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>&>(start);
                    auto g = nb::cast<const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>&>(goal);
                    pd.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ompl::base::STATE_SPACE_SO2: {
                    auto s = nb::cast<const ompl::base::ScopedState<ompl::base::SO2StateSpace>&>(start);
                    auto g = nb::cast<const ompl::base::ScopedState<ompl::base::SO2StateSpace>&>(goal);
                    pd.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ompl::base::STATE_SPACE_SO3: {
                    auto s = nb::cast<const ompl::base::ScopedState<ompl::base::SO3StateSpace>&>(start);
                    auto g = nb::cast<const ompl::base::ScopedState<ompl::base::SO3StateSpace>&>(goal);
                    pd.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ompl::base::STATE_SPACE_SE2: {
                    auto s = nb::cast<const ompl::base::ScopedState<ompl::base::SE2StateSpace>&>(start);
                    auto g = nb::cast<const ompl::base::ScopedState<ompl::base::SE2StateSpace>&>(goal);
                    pd.setStartAndGoalStates(s, g, threshold);
                    break;
                }
                case ompl::base::STATE_SPACE_SE3: {
                    auto s = nb::cast<const ompl::base::ScopedState<ompl::base::SE3StateSpace>&>(start);
                    auto g = nb::cast<const ompl::base::ScopedState<ompl::base::SE3StateSpace>&>(goal);
                    pd.setStartAndGoalStates(s, g, threshold);
                    break;
                }
               //  case ompl::base::STATE_SPACE_TIME: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::TimeStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::TimeStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_DISCRETE: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::DiscreteStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::DiscreteStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_DUBINS: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::DubinsStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::DubinsStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_REEDS_SHEPP: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_MOBIUS: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::MobiusStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::MobiusStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_SPHERE: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::SphereStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::SphereStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_TORUS: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::TorusStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::TorusStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_KLEIN_BOTTLE: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::KleinBottleStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::KleinBottleStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_VANA: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::VanaStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::VanaStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_OWEN: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::OwenStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::OwenStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
               //  case ompl::base::STATE_SPACE_VANA_OWEN: {
               //      auto s = nb::cast<const ompl::base::ScopedState<ompl::base::VanaOwenStateSpace>&>(start);
               //      auto g = nb::cast<const ompl::base::ScopedState<ompl::base::VanaOwenStateSpace>&>(goal);
               //      pd.setStartAndGoalStates(s, g, threshold);
               //      break;
               //  }
                default:
                    throw std::runtime_error("Unsupported state space type in setStartAndGoalStates");
            }
        }, nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        
        // setGoalState (ScopedState<> version)
        .def("setGoalState", nb::overload_cast<const ompl::base::ScopedState<>&, double>
             (&ompl::base::ProblemDefinition::setGoalState),
             nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())

        // Optimization Objective functions.
        .def("hasOptimizationObjective", &ompl::base::ProblemDefinition::hasOptimizationObjective)
        .def("getOptimizationObjective", &ompl::base::ProblemDefinition::getOptimizationObjective,
             nb::rv_policy::reference_internal)
        .def("setOptimizationObjective", &ompl::base::ProblemDefinition::setOptimizationObjective)

        // Intermediate Solution Callback.
        .def("getIntermediateSolutionCallback", &ompl::base::ProblemDefinition::getIntermediateSolutionCallback,
             nb::rv_policy::reference_internal)
        .def("setIntermediateSolutionCallback", &ompl::base::ProblemDefinition::setIntermediateSolutionCallback)

        // isTrivial: Bind a version that takes pointer parameters.
        .def("isTrivial", nb::overload_cast<unsigned int*, double*>
             (&ompl::base::ProblemDefinition::isTrivial, nb::const_))
        // Also provide a simpler overload that calls isTrivial with nullptr for outputs.
        .def("isTrivial", [](const ompl::base::ProblemDefinition &pd) {
            return pd.isTrivial(nullptr, nullptr);
        })

        // isStraightLinePathValid returns a path pointer.
        .def("isStraightLinePathValid", &ompl::base::ProblemDefinition::isStraightLinePathValid,
             nb::rv_policy::reference_internal)

        // fixInvalidInputStates
        .def("fixInvalidInputStates", &ompl::base::ProblemDefinition::fixInvalidInputStates)

        // Solution properties.
        .def("hasSolution", &ompl::base::ProblemDefinition::hasSolution)
        .def("hasExactSolution", &ompl::base::ProblemDefinition::hasExactSolution)
        .def("hasApproximateSolution", &ompl::base::ProblemDefinition::hasApproximateSolution)
        .def("getSolutionDifference", &ompl::base::ProblemDefinition::getSolutionDifference)
        .def("hasOptimizedSolution", &ompl::base::ProblemDefinition::hasOptimizedSolution)
        .def("getSolutionPath", &ompl::base::ProblemDefinition::getSolutionPath,
             nb::rv_policy::reference_internal)

        // getSolution: because the original function fills a PlannerSolution passed by reference,
        // we wrap it so that it returns the solution if one is found.
        .def("getSolution", &ompl::base::ProblemDefinition::getSolution, 
             nb::rv_policy::reference_internal)

        // addSolutionPath overloads.
        .def("addSolutionPath", nb::overload_cast<const ompl::base::PathPtr&, bool, double, const std::string &>
             (&ompl::base::ProblemDefinition::addSolutionPath, nb::const_),
             nb::arg("path"), nb::arg("approximate") = false, nb::arg("difference") = -1.0,
             nb::arg("plannerName") = "Unknown")
        .def("addSolutionPath", nb::overload_cast<const ompl::base::PlannerSolution &>
             (&ompl::base::ProblemDefinition::addSolutionPath, nb::const_))

        // Solution count and list.
        .def("getSolutionCount", &ompl::base::ProblemDefinition::getSolutionCount)
        .def("getSolutions", &ompl::base::ProblemDefinition::getSolutions)

        // Clear solution paths.
        .def("clearSolutionPaths", &ompl::base::ProblemDefinition::clearSolutionPaths)

        // Proof of non-existence functions.
        .def("hasSolutionNonExistenceProof", &ompl::base::ProblemDefinition::hasSolutionNonExistenceProof)
        .def("clearSolutionNonExistenceProof", &ompl::base::ProblemDefinition::clearSolutionNonExistenceProof)
        .def("getSolutionNonExistenceProof", &ompl::base::ProblemDefinition::getSolutionNonExistenceProof,
             nb::rv_policy::reference_internal)
        .def("setSolutionNonExistenceProof", &ompl::base::ProblemDefinition::setSolutionNonExistenceProof)

        // print: Wrap the print function to output to std::cout.
        .def("print", [](const ompl::base::ProblemDefinition &pd) {
            pd.print(std::cout);
        });
}