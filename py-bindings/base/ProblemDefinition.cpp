#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <optional>
#include <sstream>

// Include the OMPL headers for ProblemDefinition and related classes.
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/Goal.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SolutionNonExistenceProof.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;

namespace
{
    int problemDefinitionTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        const auto &pdef = *nb::inst_ptr<ompl::base::ProblemDefinition>(self);
        if (int rc = gc::visitHeld(pdef.getSpaceInformation(), visit, arg))
            return rc;
        if (int rc = gc::visitHeld(pdef.getGoal(), visit, arg))
            return rc;
        if (int rc = gc::visitHeld(pdef.getOptimizationObjective(), visit, arg))
            return rc;
        return gc::visitHeld(pdef.getIntermediateSolutionCallback(), visit, arg);
    }

    int problemDefinitionClear(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        auto &pdef = *nb::inst_ptr<ompl::base::ProblemDefinition>(self);
        if (gc::holdsPython(pdef.getGoal()))
            pdef.clearGoal();
        if (gc::holdsPython(pdef.getOptimizationObjective()))
            pdef.setOptimizationObjective(ompl::base::OptimizationObjectivePtr());
        if (gc::holdsPython(pdef.getIntermediateSolutionCallback()))
            pdef.setIntermediateSolutionCallback(ompl::base::ReportIntermediateSolutionFn());
        return 0;
    }

    const PyType_Slot problemDefinitionSlots[] = {{Py_tp_traverse, (void *)problemDefinitionTraverse},
                                                  {Py_tp_clear, (void *)problemDefinitionClear},
                                                  {0, 0}};
}  // namespace

void ompl::binding::base::init_ProblemDefinition(nb::module_ &m)
{
    nb::class_<ompl::base::PlannerSolution>(m, "PlannerSolution")
        .def(nb::init<const ompl::base::PathPtr &>(), nb::arg("path"))
        .def_rw("index_", &ompl::base::PlannerSolution::index_)
        .def_rw("path_", &ompl::base::PlannerSolution::path_)
        .def_rw("length_", &ompl::base::PlannerSolution::length_)
        .def_rw("approximate_", &ompl::base::PlannerSolution::approximate_)
        .def_rw("difference_", &ompl::base::PlannerSolution::difference_)
        .def_rw("optimized_", &ompl::base::PlannerSolution::optimized_)
        .def_rw("opt_", &ompl::base::PlannerSolution::opt_)
        .def_rw("cost_", &ompl::base::PlannerSolution::cost_)
        .def_rw("plannerName_", &ompl::base::PlannerSolution::plannerName_)
        .def("setApproximate", &ompl::base::PlannerSolution::setApproximate, nb::arg("difference"))
        .def("setOptimized", &ompl::base::PlannerSolution::setOptimized, nb::arg("opt"), nb::arg("cost"),
             nb::arg("meetsObjective"))
        .def("setPlannerName", &ompl::base::PlannerSolution::setPlannerName, nb::arg("name"))
        .def("__eq__", &ompl::base::PlannerSolution::operator==, nb::arg("other"))
        .def("__lt__", &ompl::base::PlannerSolution::operator<, nb::arg("other"));

    nb::class_<ompl::base::ProblemDefinition>(m, "ProblemDefinition", nb::type_slots(problemDefinitionSlots))
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

        .def("hasStartState", [](const ompl::base::ProblemDefinition &pd, const ompl::base::State *state) {
            return pd.hasStartState(state);
        }, nb::arg("state"))
        .def("hasStartStateWithIndex", [](const ompl::base::ProblemDefinition &pd, const ompl::base::State *state) {
            unsigned int index = 0;
            bool found = pd.hasStartState(state, &index);
            return std::make_pair(found, found ? std::optional<unsigned int>(index) : std::nullopt);
        }, nb::arg("state"))

        // clearStartStates
        .def("clearStartStates", &ompl::base::ProblemDefinition::clearStartStates)

        // getStartStateCount
        .def("getStartStateCount", &ompl::base::ProblemDefinition::getStartStateCount)

        // getStartState overloads.
        .def("getStartState", nb::overload_cast<unsigned int>
             (&ompl::base::ProblemDefinition::getStartState, nb::const_), nb::rv_policy::reference_internal)
        .def("getStartState", nb::overload_cast<unsigned int>
             (&ompl::base::ProblemDefinition::getStartState), nb::rv_policy::reference_internal)

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
        }, nb::rv_policy::reference_internal)

        // setStartAndGoalStates (State* version) with default threshold.
        .def("setStartAndGoalStates", nb::overload_cast<const ompl::base::State*, const ompl::base::State*, double>
             (&ompl::base::ProblemDefinition::setStartAndGoalStates),
             nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())

        // setGoalState (State* version) with default threshold.
        .def("setGoalState", nb::overload_cast<const ompl::base::State*, double>
          (&ompl::base::ProblemDefinition::setGoalState),
          nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())

        // Optimization Objective functions.
        .def("hasOptimizationObjective", &ompl::base::ProblemDefinition::hasOptimizationObjective)
        .def("getOptimizationObjective", &ompl::base::ProblemDefinition::getOptimizationObjective,
             nb::rv_policy::reference_internal)
        .def("setOptimizationObjective", &ompl::base::ProblemDefinition::setOptimizationObjective)

        // Intermediate Solution Callback.
        .def("getIntermediateSolutionCallback", [](const ompl::base::ProblemDefinition &pd) -> nb::object {
            const auto &callback = pd.getIntermediateSolutionCallback();
            if (nb::handle fn = gc::callableOf(callback); fn.is_valid())
                return nb::borrow(fn);
            return callback ? nb::cast(callback) : nb::none();
        })
        .def("setIntermediateSolutionCallback", [](ompl::base::ProblemDefinition &pd, nb::callable callback) {
            pd.setIntermediateSolutionCallback(
                gc::function<ompl::base::ReportIntermediateSolutionFn>(std::move(callback)));
        }, nb::arg("callback"))
        .def("setIntermediateSolutionCallback", [](ompl::base::ProblemDefinition &pd, std::nullptr_t) {
            pd.setIntermediateSolutionCallback(ompl::base::ReportIntermediateSolutionFn());
        }, nb::arg("callback").none())

        // isTrivial: report the start index and distance the pointer arguments would have written.
        .def("isTrivialWithDetails", [](const ompl::base::ProblemDefinition &pd) {
            unsigned int startIndex = 0;
            double distance = 0.0;
            bool trivial = pd.isTrivial(&startIndex, &distance);
            return std::make_tuple(trivial, startIndex, distance);
        })
        // Also provide a simpler overload that calls isTrivial with nullptr for outputs.
        .def("isTrivial", [](const ompl::base::ProblemDefinition &pd) {
            return pd.isTrivial(nullptr, nullptr);
        })

        // isStraightLinePathValid returns a path pointer.
        .def("isStraightLinePathValid", &ompl::base::ProblemDefinition::isStraightLinePathValid)

        // fixInvalidInputStates
        .def("fixInvalidInputStates", &ompl::base::ProblemDefinition::fixInvalidInputStates)

        // Solution properties.
        .def("hasSolution", &ompl::base::ProblemDefinition::hasSolution)
        .def("hasExactSolution", &ompl::base::ProblemDefinition::hasExactSolution)
        .def("hasApproximateSolution", &ompl::base::ProblemDefinition::hasApproximateSolution)
        .def("getSolutionDifference", &ompl::base::ProblemDefinition::getSolutionDifference)
        .def("hasOptimizedSolution", &ompl::base::ProblemDefinition::hasOptimizedSolution)
        .def("getSolutionPath", &ompl::base::ProblemDefinition::getSolutionPath)

        // getSolution: because the original function fills a PlannerSolution passed by reference,
        // we wrap it so that it returns the solution if one is found.
        .def("getSolution", [](const ompl::base::ProblemDefinition &pd) -> std::optional<ompl::base::PlannerSolution> {
            ompl::base::PlannerSolution solution(nullptr);
            if (pd.getSolution(solution))
                return solution;
            return std::nullopt;
        })

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
        .def("getSolutionNonExistenceProof", &ompl::base::ProblemDefinition::getSolutionNonExistenceProof)
        .def("setSolutionNonExistenceProof", &ompl::base::ProblemDefinition::setSolutionNonExistenceProof)

        // print: Wrap the print function to output to std::cout.
        .def("print", [](const ompl::base::ProblemDefinition &pd) {
            pd.print(std::cout);
        })
        .def("__repr__", [](const ompl::base::ProblemDefinition &pd) {
            std::ostringstream oss;
            pd.print(oss);
            return oss.str();
        });
}
