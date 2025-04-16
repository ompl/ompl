#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerStatus.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Planner(nb::module_ &m)
{
    struct PyPlanner : ob::Planner
    {
        // We declare an NB_TRAMPOLINE for 8 override slots (the number of virtual methods we plan to override).
        NB_TRAMPOLINE(ob::Planner, 8);

        // The pure virtual function: solve(...)
        ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override
        {
            NB_OVERRIDE_PURE(solve, ptc);
        }

        // The rest of the virtual functions:
        void clear() override
        {
            NB_OVERRIDE(clear);
        }

        void clearQuery() override
        {
            NB_OVERRIDE(clearQuery);
        }

        void getPlannerData(ob::PlannerData &data) const override
        {
            NB_OVERRIDE(getPlannerData, data);
        }

        void setup() override
        {
            NB_OVERRIDE(setup);
        }

        void checkValidity() override
        {
            NB_OVERRIDE(checkValidity);
        }

        void printProperties(std::ostream &out) const override
        {
            NB_OVERRIDE(printProperties, out);
        }

        void printSettings(std::ostream &out) const override
        {
            NB_OVERRIDE(printSettings, out);
        }
    };
    //     nb::class_<ompl::base::Planner>(m, "_Planner");

    nb::class_<ob::Planner, PyPlanner /* <-- trampoline */>(m, "Planner")
        // Constructors
        .def(nb::init<ob::SpaceInformationPtr, std::string>())
        // Expose additional methods
        .def(
            "getSpaceInformation", [](ob::Planner &p) -> ob::SpaceInformationPtr { return p.getSpaceInformation(); },
            nb::rv_policy::reference_internal, "Return the SpaceInformation associated with this planner.")
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(ob::Planner::*)()>(&ob::Planner::getProblemDefinition),
             nb::rv_policy::reference_internal, "Return a reference to the ProblemDefinition (non-const).")
        .def("setProblemDefinition", &ob::Planner::setProblemDefinition, nb::arg("pdef"),
             "Set the ProblemDefinition for this planner.")
        .def(
            "solve", [](ob::Planner &pl, double solveTime) { return pl.solve(solveTime); }, nb::arg("solveTime"),
            "Attempt to solve for a given time (seconds).")
        .def(
            "solve", [](ob::Planner &pl, const ob::PlannerTerminationConditionFn &fn, double checkInterval)
            { return pl.solve(fn, checkInterval); }, nb::arg("terminationConditionFn"), nb::arg("checkInterval") = 0.0,
            "Attempt to solve with a termination condition function.")
        .def("getName", &ob::Planner::getName, "Return the planner's name.")
        .def("setName", &ob::Planner::setName, nb::arg("name"), "Set the planner's name.")
        .def("getSpecs", &ob::Planner::getSpecs, nb::rv_policy::reference_internal,
             "Return the PlannerSpecs describing the planner's capabilities.")
        .def("params", static_cast<ob::ParamSet &(ob::Planner::*)()>(&ob::Planner::params),
             nb::rv_policy::reference_internal, "Return a reference to the planner's ParamSet (modifiable).")
        .def("paramsConst", static_cast<const ob::ParamSet &(ob::Planner::*)() const>(&ob::Planner::params),
             nb::rv_policy::reference_internal, "Return a constant reference to the planner's ParamSet.")
        .def("getPlannerProgressProperties", &ob::Planner::getPlannerProgressProperties,
             nb::rv_policy::reference_internal, "Return a map of property names to progress property functions.");

    nb::class_<ompl::base::PlannerInputStates>(m, "PlannerInputStates",
                                               "Wrapper class to maintain the set of input states that a planner can "
                                               "use")
        // Constructors
        .def(nb::init<const ompl::base::PlannerPtr &>(), "Constructor that takes a planner instance")
        .def(nb::init<const ompl::base::Planner *>(), "Constructor that takes a planner pointer")
        .def(nb::init<>(), "Default constructor")

        // Core functionality
        .def("clear", &ompl::base::PlannerInputStates::clear, "Clear all input states")
        .def("restart", &ompl::base::PlannerInputStates::restart,
             "Start the iteration over input states from the beginning")
        .def("update", &ompl::base::PlannerInputStates::update, "Update the set of input states")
        .def("use", &ompl::base::PlannerInputStates::use, "Use the states from the specified problem definition")
        .def("checkValidity", &ompl::base::PlannerInputStates::checkValidity, "Check if the input states are valid")

        // State iteration
        .def("nextStart", &ompl::base::PlannerInputStates::nextStart, "Get the next start state")
        .def("nextGoal", nb::overload_cast<>(&ompl::base::PlannerInputStates::nextGoal), "Get the next goal state")
        .def("nextGoal",
             nb::overload_cast<const ompl::base::PlannerTerminationCondition &>(
                 &ompl::base::PlannerInputStates::nextGoal),
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
