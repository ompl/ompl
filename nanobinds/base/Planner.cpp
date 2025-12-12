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
    nb::class_<ompl::base::PlannerInputStates>(m, "PlannerInputStates")
        // Constructors
        .def(nb::init<const ompl::base::PlannerPtr &>())
        .def(nb::init<const ompl::base::Planner *>())
        .def(nb::init<>())

        // Core functionality
        .def("clear", &ompl::base::PlannerInputStates::clear)
        .def("restart", &ompl::base::PlannerInputStates::restart)
        .def("update", &ompl::base::PlannerInputStates::update)
        .def("use", &ompl::base::PlannerInputStates::use)
        .def("checkValidity", &ompl::base::PlannerInputStates::checkValidity)

        // State iteration
        .def("nextStart", &ompl::base::PlannerInputStates::nextStart,nb::rv_policy::reference_internal)
        .def("nextGoal", nb::overload_cast<>(&ompl::base::PlannerInputStates::nextGoal), nb::rv_policy::reference_internal)
        .def("nextGoal",
             nb::overload_cast<const ompl::base::PlannerTerminationCondition &>(
                 &ompl::base::PlannerInputStates::nextGoal), nb::rv_policy::reference_internal)

        // State availability checks
        .def("haveMoreStartStates", &ompl::base::PlannerInputStates::haveMoreStartStates)
        .def("haveMoreGoalStates", &ompl::base::PlannerInputStates::haveMoreGoalStates)

        // State counting
        .def("getSeenStartStatesCount", &ompl::base::PlannerInputStates::getSeenStartStatesCount)
        .def("getSampledGoalsCount", &ompl::base::PlannerInputStates::getSampledGoalsCount);
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

    nb::class_<ob::Planner, PyPlanner /* <-- trampoline */>(m, "Planner")
        // Constructors
        .def(nb::init<ob::SpaceInformationPtr, std::string>())
        .def(
            "getSpaceInformation", [](ob::Planner &p) -> ob::SpaceInformationPtr { return p.getSpaceInformation(); },
            nb::rv_policy::reference_internal)
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(ob::Planner::*)()>(&ob::Planner::getProblemDefinition),
             nb::rv_policy::reference_internal)
        .def("setProblemDefinition", &ob::Planner::setProblemDefinition, nb::arg("pdef"))
        .def ("getPlannerInputStates", &ob::Planner::getPlannerInputStates,
              nb::rv_policy::reference_internal)
        .def(
            "solve", [](ob::Planner &pl, double solveTime) { return pl.solve(solveTime); }, nb::arg("solveTime"))
        .def(
            "solve", [](ob::Planner &pl, const ob::PlannerTerminationConditionFn &fn, double checkInterval)
            { return pl.solve(fn, checkInterval); }, nb::arg("terminationConditionFn"), nb::arg("checkInterval") = 0.0)
        .def("clearQuery", &ob::Planner::clearQuery)
        .def("clear", &ob::Planner::clear)
        .def("getName", &ob::Planner::getName)
        .def("setName", &ob::Planner::setName, nb::arg("name"))
        .def("getSpecs", &ob::Planner::getSpecs, nb::rv_policy::reference_internal)
        .def("params", static_cast<ob::ParamSet &(ob::Planner::*)()>(&ob::Planner::params),
             nb::rv_policy::reference_internal)
        .def("paramsConst", static_cast<const ob::ParamSet &(ob::Planner::*)() const>(&ob::Planner::params),
             nb::rv_policy::reference_internal)
        .def("getPlannerProgressProperties", &ob::Planner::getPlannerProgressProperties,
             nb::rv_policy::reference_internal);
}
