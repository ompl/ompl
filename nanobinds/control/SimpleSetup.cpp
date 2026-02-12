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

#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

struct SimpleSetupPublicist : public oc::SimpleSetup
{
    using oc::SimpleSetup::configured_;
};

// function decl not needed here since we are defining it
// void ompl::binding::control::init_SimpleSetup(nb::module_ &m);

// Helper dictionary access
static PyObject **get_dict_ptr(PyObject *obj)
{
    return _PyObject_GetDictPtr(obj);
}

int simple_setup_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;

    // 1. Visit __dict__
    PyObject **dictptr = get_dict_ptr(self);
    if (dictptr && *dictptr)
    {
        Py_VISIT(*dictptr);
    }

    try
    {
        auto *ss = nb::inst_ptr<oc::SimpleSetup>(self);
        if (ss)
        {
            // Visit SpaceInformation
            auto si = ss->getSpaceInformation();
            if (si)
            {
                nb::handle h = nb::find(si);
                if (h.is_valid())
                    Py_VISIT(h.ptr());
            }
            // Visit ProblemDefinition
            auto pdef = ss->getProblemDefinition();
            if (pdef)
            {
                nb::handle h = nb::find(pdef);
                if (h.is_valid())
                    Py_VISIT(h.ptr());
            }
            // Visit Planner
            auto planner = ss->getPlanner();
            if (planner)
            {
                nb::handle h = nb::find(planner);
                if (h.is_valid())
                    Py_VISIT(h.ptr());
            }
            // Visit Callbacks stored in dict (if nb::find fails for them, though SS delegates to SI usually)
            // But SS also overrides setStateValidityChecker.
            if (dictptr && *dictptr)
            {
                PyObject *svc = PyDict_GetItemString(*dictptr, "_svc");
                if (svc)
                    Py_VISIT(svc);
                PyObject *prop = PyDict_GetItemString(*dictptr, "_prop");
                if (prop)
                    Py_VISIT(prop);
            }
        }
    }
    catch (...)
    {
    }
    return 0;
}

int simple_setup_tp_clear(PyObject *self)
{
    // 1. Clear __dict__
    PyObject **dictptr = get_dict_ptr(self);
    if (dictptr && *dictptr)
    {
        Py_CLEAR(*dictptr);
    }
    // 2. Break C++ Cycle
    try
    {
        auto *ss = nb::inst_ptr<oc::SimpleSetup>(self);
        if (ss)
        {
            ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(nullptr));
            ss->setStatePropagator(oc::StatePropagatorPtr(nullptr));
        }
    }
    catch (...)
    {
    }
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
        .def("getPlannerAllocator", &oc::SimpleSetup::getPlannerAllocator)

        // haveExactSolutionPath, haveSolutionPath
        .def("haveExactSolutionPath", &oc::SimpleSetup::haveExactSolutionPath)
        .def("haveSolutionPath", &oc::SimpleSetup::haveSolutionPath)

        // getSolutionPath (return by reference)
        .def(
            "getSolutionPath", [](oc::SimpleSetup &self) -> oc::PathControl & { return self.getSolutionPath(); },
            nb::rv_policy::reference_internal)

        // getPlannerData
        .def(
            "getPlannerData", [](oc::SimpleSetup &self, ob::PlannerData &pd) { self.getPlannerData(pd); },
            nb::arg("plannerData"))

        // setStateValidityChecker (two overloads)
        .def(
            "setStateValidityChecker",
            [](oc::SimpleSetup &ss, const ompl::base::StateValidityCheckerFn &svc)
            {
                ss.setStateValidityChecker(svc);
                nb::object self = nb::find(nb::cast(&ss));
                if (self.is_valid())
                    nb::setattr(self, "_svc", nb::cast(svc));
            },
            nb::arg("svc"))
        .def(
            "setStateValidityChecker",
            [](oc::SimpleSetup &ss, const ompl::base::StateValidityCheckerPtr &svc)
            {
                ss.setStateValidityChecker(svc);
                nb::object self = nb::find(nb::cast(&ss));
                if (self.is_valid())
                    nb::setattr(self, "_svc", nb::cast(svc));
            },
            nb::arg("svc"))

        // setStatePropagator (Ptr and Fn)
        .def(
            "setStatePropagator",
            [](oc::SimpleSetup &ss, const oc::StatePropagatorPtr &sp)
            {
                ss.setStatePropagator(sp);
                nb::object self = nb::find(nb::cast(&ss));
                if (self.is_valid())
                    nb::setattr(self, "_prop", nb::cast(sp));
            },
            nb::arg("sp"))
        .def(
            "setStatePropagator",
            [](oc::SimpleSetup &ss, const oc::StatePropagatorFn &sp)
            {
                ss.setStatePropagator(sp);
                nb::object self = nb::find(nb::cast(&ss));
                if (self.is_valid())
                    nb::setattr(self, "_prop", nb::cast(sp));
            },
            nb::arg("sp"))

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
        .def("setPlannerAllocator", &oc::SimpleSetup::setPlannerAllocator, nb::arg("allocator"))

        // solve() methods (two overloads)
        .def("solve", nb::overload_cast<double>(&oc::SimpleSetup::solve), nb::arg("time") = 1.0)
        .def("solve", nb::overload_cast<const ob::PlannerTerminationCondition &>(&oc::SimpleSetup::solve),
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
        .def("__str__",
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
