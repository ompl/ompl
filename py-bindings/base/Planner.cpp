#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/map.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>
#include <map>
#include <sstream>

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerStatus.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;

namespace
{
    int plannerTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return ompl::binding::base::visitParams(nb::inst_ptr<ob::Planner>(self)->params(), visit, arg);
    }

    int plannerClear(PyObject *self)
    {
        if (nb::inst_ready(self) && gc::pythonOwned(self))
            ompl::binding::base::clearParams(nb::inst_ptr<ob::Planner>(self)->params());
        return 0;
    }

    const PyType_Slot plannerSlots[] = {{Py_tp_traverse, (void *)plannerTraverse},
                                        {Py_tp_clear, (void *)plannerClear},
                                        {0, 0}};
}  // namespace

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
        .def("nextStart", &ompl::base::PlannerInputStates::nextStart, nb::rv_policy::reference_internal)
        .def("nextGoal", nb::overload_cast<>(&ompl::base::PlannerInputStates::nextGoal),
             nb::rv_policy::reference_internal)
        // Waits for a GoalLazySamples thread, which needs the GIL to produce a goal.
        .def(
            "nextGoal",
            [](ompl::base::PlannerInputStates &pis, const ompl::base::PlannerTerminationCondition &ptc)
            {
                nb::gil_scoped_release release;
                return pis.nextGoal(ptc);
            },
            nb::arg("ptc"), nb::rv_policy::reference_internal)

        // State availability checks
        .def("haveMoreStartStates", &ompl::base::PlannerInputStates::haveMoreStartStates)
        .def("haveMoreGoalStates", &ompl::base::PlannerInputStates::haveMoreGoalStates)

        // State counting
        .def("getSeenStartStatesCount", &ompl::base::PlannerInputStates::getSeenStartStatesCount)
        .def("getSampledGoalsCount", &ompl::base::PlannerInputStates::getSampledGoalsCount);
    struct PyPlanner : ob::Planner
    {
        // We declare an NB_TRAMPOLINE for 6 override slots (the number of virtual methods we plan to override).
        // printProperties()/printSettings() are not among them: std::ostream has no caster, so a Python override
        // could never be called.
        NB_TRAMPOLINE(ob::Planner, 6);

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

        // Passed by reference: NB_OVERRIDE would copy it, and PlannerData is not copyable.
        void getPlannerData(ob::PlannerData &data) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "getPlannerData", false);
            if (!nb_ticket.key.is_valid())
                return ob::Planner::getPlannerData(data);
            nb_trampoline.base().attr(nb_ticket.key)(nb::cast(&data, nb::rv_policy::reference));
        }

        void setup() override
        {
            NB_OVERRIDE(setup);
        }

        void checkValidity() override
        {
            NB_OVERRIDE(checkValidity);
        }
    };

    nb::class_<ob::Planner, PyPlanner /* <-- trampoline */>(m, "Planner", nb::type_slots(plannerSlots))
        // Constructors
        .def(nb::init<ob::SpaceInformationPtr, std::string>())
        .def("getSpaceInformation", [](ob::Planner &p) -> ob::SpaceInformationPtr { return p.getSpaceInformation(); })
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(ob::Planner::*)()>(&ob::Planner::getProblemDefinition))
        .def("setProblemDefinition", &ob::Planner::setProblemDefinition, nb::arg("pdef"))
        .def("getPlannerInputStates", &ob::Planner::getPlannerInputStates, nb::rv_policy::reference_internal)
        // A PlannerTerminationCondition is itself callable, so its overload has to come before the callable one.
        .def(
            "solve",
            [](ob::Planner &pl, const ob::PlannerTerminationCondition &ptc)
            {
                nb::gil_scoped_release release;
                return pl.solve(ptc);
            },
            nb::arg("ptc"))
        .def(
            "solve",
            [](ob::Planner &pl, double solveTime)
            {
                nb::gil_scoped_release release;
                return pl.solve(solveTime);
            },
            nb::arg("solveTime"))
        .def(
            "solve",
            [](ob::Planner &pl, const ob::PlannerTerminationConditionFn &fn, double checkInterval)
            {
                nb::gil_scoped_release release;
                return pl.solve(fn, checkInterval);
            },
            nb::arg("terminationConditionFn"), nb::arg("checkInterval") = 0.0)
        .def("clearQuery", &ob::Planner::clearQuery)
        .def("clear", &ob::Planner::clear)
        .def("setup", &ob::Planner::setup)
        .def("checkValidity", &ob::Planner::checkValidity)
        .def("getName", &ob::Planner::getName)
        .def("setName", &ob::Planner::setName, nb::arg("name"))
        .def("getSpecs", &ob::Planner::getSpecs, nb::rv_policy::reference_internal)
        .def("getPlannerData", &ob::Planner::getPlannerData, nb::arg("data"))
        .def("params", static_cast<ob::ParamSet &(ob::Planner::*)()>(&ob::Planner::params),
             nb::rv_policy::reference_internal)
        // Evaluated here: the property functions capture the planner's `this`, so handing them out would dangle.
        .def("getPlannerProgressProperties",
             [](const ob::Planner &p)
             {
                 std::map<std::string, std::string> values;
                 for (const auto &[name, property] : p.getPlannerProgressProperties())
                     values[name] = property();
                 return values;
             })
        .def("printProperties",
             [](const ob::Planner &p)
             {
                 std::ostringstream oss;
                 p.printProperties(oss);
                 return oss.str();
             })
        .def("printSettings",
             [](const ob::Planner &p)
             {
                 std::ostringstream oss;
                 p.printSettings(oss);
                 return oss.str();
             });

    nb::class_<ob::PlannerSpecs>(m, "PlannerSpecs")
        .def(nb::init<>())
        .def_rw("recognizedGoal", &ob::PlannerSpecs::recognizedGoal)
        .def_rw("multithreaded", &ob::PlannerSpecs::multithreaded)
        .def_rw("approximateSolutions", &ob::PlannerSpecs::approximateSolutions)
        .def_rw("optimizingPaths", &ob::PlannerSpecs::optimizingPaths)
        .def_rw("directed", &ob::PlannerSpecs::directed)
        .def_rw("provingSolutionNonExistence", &ob::PlannerSpecs::provingSolutionNonExistence)
        .def_rw("canReportIntermediateSolutions", &ob::PlannerSpecs::canReportIntermediateSolutions);
}
