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
#include "ompl/util/Exception.h"

#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace oc = ompl::control;
namespace ob = ompl::base;

struct SimpleSetupPublicist : public oc::SimpleSetup
{
    using oc::SimpleSetup::configured_;
};

// The SI and pdef are shared with the planner and the user, so only what ss alone holds counts.
int simple_setup_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;
    if (int rc = gc::visitDict(self, visit, arg))
        return rc;
    if (!gc::pythonOwned(self))
        return 0;

    const auto &ss = *nb::inst_ptr<oc::SimpleSetup>(self);
    if (int rc = gc::visitHeld(ss.getPlanner(), visit, arg))
        return rc;
    return gc::visitHeld(ss.getPlannerAllocator(), visit, arg);
}

int simple_setup_tp_clear(PyObject *self)
{
    if (nb::inst_ready(self) && gc::pythonOwned(self))
    {
        auto &ss = *nb::inst_ptr<oc::SimpleSetup>(self);
        if (gc::holdsPython(ss.getPlannerAllocator()))
            ss.setPlannerAllocator(ob::PlannerAllocator());
        if (gc::holdsPython(ss.getPlanner()))
            ss.setPlanner(ob::PlannerPtr());
    }
    gc::clearDict(self);
    return 0;
}

PyType_Slot simple_setup_slots[] = {{Py_tp_traverse, (void *)simple_setup_tp_traverse},
                                    {Py_tp_clear, (void *)simple_setup_tp_clear},
                                    {0, 0}};

void ompl::binding::control::init_SimpleSetup(nb::module_ &m)
{
    nb::class_<oc::SimpleSetup>(m, "SimpleSetup", nb::type_slots(simple_setup_slots), nb::dynamic_attr())
        // Constructors
        .def(nb::init<oc::SpaceInformationPtr>(), nb::arg("spaceInfo"))
        .def(nb::init<oc::ControlSpacePtr>(), nb::arg("controlSpace"))

        // getSpaceInformation
        .def("getSpaceInformation", &oc::SimpleSetup::getSpaceInformation)

        // getProblemDefinition
        .def("getProblemDefinition",
             static_cast<ob::ProblemDefinitionPtr &(oc::SimpleSetup::*)()>(&oc::SimpleSetup::getProblemDefinition))

        // getStateSpace
        .def("getStateSpace", &oc::SimpleSetup::getStateSpace)

        // getControlSpace
        .def("getControlSpace", &oc::SimpleSetup::getControlSpace)

        // getStateValidityChecker
        .def("getStateValidityChecker", &oc::SimpleSetup::getStateValidityChecker)

        // getStatePropagator
        .def("getStatePropagator", &oc::SimpleSetup::getStatePropagator)

        // getGoal
        .def("getGoal", &oc::SimpleSetup::getGoal)

        // getPlanner
        .def("getPlanner", &oc::SimpleSetup::getPlanner)

        // getPlannerAllocator
        .def("getPlannerAllocator",
             [](const oc::SimpleSetup &ss) -> nb::object
             {
                 const ob::PlannerAllocator &pa = ss.getPlannerAllocator();
                 if (nb::handle fn = gc::callableOf(pa); fn.is_valid())
                     return nb::borrow(fn);
                 return pa ? nb::cast(pa) : nb::none();
             })

        // haveExactSolutionPath, haveSolutionPath
        .def("haveExactSolutionPath", &oc::SimpleSetup::haveExactSolutionPath)
        .def("haveSolutionPath", &oc::SimpleSetup::haveSolutionPath)

        // Shares ownership: clear(), setStartAndGoalStates() and solve() drop the problem definition's path.
        .def("getSolutionPath",
             [](const oc::SimpleSetup &self)
             {
                 const ob::ProblemDefinitionPtr &pdef = self.getProblemDefinition();
                 auto path = pdef ? std::dynamic_pointer_cast<oc::PathControl>(pdef->getSolutionPath()) : nullptr;
                 if (!path)
                     throw ompl::Exception("No solution path");
                 return path;
             })

        // getPlannerData
        .def(
            "getPlannerData", [](oc::SimpleSetup &self, ob::PlannerData &pd) { self.getPlannerData(pd); },
            nb::arg("plannerData"))

        // The SpaceInformation holds the checker and the propagator, so its binding wraps callables.
        .def(
            "setStateValidityChecker", [](oc::SimpleSetup &ss, const ob::StateValidityCheckerPtr &svc)
            { ss.setStateValidityChecker(svc); }, nb::arg("svc"))
        .def(
            "setStateValidityChecker", [](oc::SimpleSetup &ss, nb::callable svc)
            { nb::cast(ss.getSpaceInformation()).attr("setStateValidityChecker")(svc); }, nb::arg("svc"))
        .def(
            "setStatePropagator",
            [](oc::SimpleSetup &ss, const oc::StatePropagatorPtr &sp) { ss.setStatePropagator(sp); }, nb::arg("sp"))
        .def(
            "setStatePropagator", [](oc::SimpleSetup &ss, nb::callable sp)
            { nb::cast(ss.getSpaceInformation()).attr("setStatePropagator")(sp); }, nb::arg("sp"))

        // setOptimizationObjective
        .def("setOptimizationObjective", &oc::SimpleSetup::setOptimizationObjective, nb::arg("objective"))

        // start/goal states
        .def(
            "setStartAndGoalStates",
            [](oc::SimpleSetup &ss, const ob::State *start, const ob::State *goal, double threshold)
            {
                ss.getProblemDefinition()->setStartAndGoalStates(start, goal, threshold);
                ss.getProblemDefinition()->clearSolutionPaths();
                static_cast<SimpleSetupPublicist &>(ss).configured_ = false;
            },
            nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def(
            "setGoalState",
            [](oc::SimpleSetup &ss, const ob::State *goal, double threshold)
            {
                ss.getProblemDefinition()->setGoalState(goal, threshold);
                static_cast<SimpleSetupPublicist &>(ss).configured_ = false;
            },
            nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def(
            "addStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                // Retrieve the state space from the SimpleSetup.
                ss.getProblemDefinition()->addStartState(state);
            },
            nb::arg("state"))
        .def("clearStartStates", &oc::SimpleSetup::clearStartStates)
        .def(
            "setStartState",
            [](oc::SimpleSetup &ss, const ob::State *state)
            {
                ss.clearStartStates();
                ss.getProblemDefinition()->addStartState(state);
            },
            nb::arg("state"))
        .def("setGoal", &oc::SimpleSetup::setGoal, nb::arg("goal"))

        // setPlanner, setPlannerAllocator
        .def("setPlanner", &oc::SimpleSetup::setPlanner, nb::arg("planner"))
        .def(
            "setPlannerAllocator", [](oc::SimpleSetup &ss, nb::callable allocator)
            { ss.setPlannerAllocator(gc::function<ob::PlannerAllocator>(std::move(allocator))); }, nb::arg("allocator"))
        .def(
            "setPlannerAllocator", [](oc::SimpleSetup &ss, std::nullptr_t)
            { ss.setPlannerAllocator(ob::PlannerAllocator()); }, nb::arg("allocator").none())

        // setup() keeps the GIL for Python callbacks that may not take it themselves; see PlannerSolve.h for solve.
        .def(
            "solve",
            [](oc::SimpleSetup &ss, double time)
            {
                ss.setup();
                nb::gil_scoped_release release;
                return ss.solve(time);
            },
            nb::arg("time") = 1.0)
        .def(
            "solve",
            [](oc::SimpleSetup &ss, const ob::PlannerTerminationCondition &ptc)
            {
                ss.setup();
                nb::gil_scoped_release release;
                return ss.solve(ptc);
            },
            nb::arg("terminationCondition"))

        // getLastPlannerStatus, getLastPlanComputationTime
        .def("getLastPlannerStatus", &oc::SimpleSetup::getLastPlannerStatus)
        .def("getLastPlanComputationTime", &oc::SimpleSetup::getLastPlanComputationTime)

        // clear
        .def("clear", &oc::SimpleSetup::clear)

        // print
        .def("print",
             [](const oc::SimpleSetup &ss)
             {
                 std::ostringstream oss;
                 ss.print(oss);
                 return oss.str();
             })
        .def("__repr__",
             [](const oc::SimpleSetup &ss)
             {
                 std::ostringstream oss;
                 ss.print(oss);
                 return oss.str();
             })

        // setup
        .def("setup", &oc::SimpleSetup::setup);
}
