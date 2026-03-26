#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/Goal.h"
#include "ompl/base/Planner.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct SimpleSetupPublicist : public og::SimpleSetup
{
    using og::SimpleSetup::configured_;
};

void ompl::binding::geometric::init_SimpleSetup(nb::module_ &m);
// Helper dictionary access
static PyObject **get_dict_ptr(PyObject *obj)
{
    return _PyObject_GetDictPtr(obj);
}

int simple_setup_geometric_tp_traverse(PyObject *self, visitproc visit, void *arg)
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
        auto *ss = nb::inst_ptr<og::SimpleSetup>(self);
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
            // Visit Callbacks stored in dict (if nb::find fails for them)
            if (dictptr && *dictptr)
            {
                PyObject *svc = PyDict_GetItemString(*dictptr, "_svc");
                if (svc)
                    Py_VISIT(svc);
            }
        }
    }
    catch (...)
    {
    }
    return 0;
}

int simple_setup_geometric_tp_clear(PyObject *self)
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
        auto *ss = nb::inst_ptr<og::SimpleSetup>(self);
        if (ss)
        {
            ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(nullptr));
        }
    }
    catch (...)
    {
    }
    return 0;
}

PyType_Slot simple_setup_geometric_slots[] = {{Py_tp_traverse, (void *)simple_setup_geometric_tp_traverse},
                                              {Py_tp_clear, (void *)simple_setup_geometric_tp_clear},
                                              {0, 0}};

void ompl::binding::geometric::init_SimpleSetup(nb::module_ &m)
{
    nb::class_<og::SimpleSetup>(m, "SimpleSetup", nb::type_slots(simple_setup_geometric_slots), nb::dynamic_attr())
     // Constructors
     .def(nb::init<const ob::SpaceInformationPtr &>(),
          nb::arg("si"))
     .def(nb::init<const ob::StateSpacePtr &>(),
          nb::arg("space"))
     
     // getSpaceInformation
     .def("getSpaceInformation",
          &og::SimpleSetup::getSpaceInformation)
     
     // getProblemDefinition (two overloads: const and non-const)
     // getProblemDefinition (two overloads: const and non-const)
     .def("getProblemDefinition",
          static_cast<ob::ProblemDefinitionPtr &(og::SimpleSetup::*)()>(&og::SimpleSetup::getProblemDefinition))
     .def("getProblemDefinitionConst",
          static_cast<const ob::ProblemDefinitionPtr &(og::SimpleSetup::*)() const>(&og::SimpleSetup::getProblemDefinition))
     
     // getStateSpace
     .def("getStateSpace",
          &og::SimpleSetup::getStateSpace)
     
     // getStateValidityChecker
     .def("getStateValidityChecker",
          &og::SimpleSetup::getStateValidityChecker)
     
     // getGoal
     .def("getGoal",
          &og::SimpleSetup::getGoal)
     
     // getPlanner
     .def("getPlanner",
          &og::SimpleSetup::getPlanner)
     
     // getPlannerAllocator
     .def("getPlannerAllocator",
          &og::SimpleSetup::getPlannerAllocator)
     
     // getPathSimplifier
     .def("getPathSimplifier",
          static_cast<og::PathSimplifierPtr &(og::SimpleSetup::*)()>(&og::SimpleSetup::getPathSimplifier))
     .def("getPathSimplifierConst",
          static_cast<const og::PathSimplifierPtr &(og::SimpleSetup::*)() const>(&og::SimpleSetup::getPathSimplifier))
     
     // getOptimizationObjective
     .def("getOptimizationObjective",
          &og::SimpleSetup::getOptimizationObjective)
     
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
     
     // setStateValidityChecker (two overloads)
     .def("setStateValidityChecker",
         [](og::SimpleSetup &ss, const ompl::base::StateValidityCheckerFn &svc) {
             ss.setStateValidityChecker(svc);
             nb::object self = nb::find(nb::cast(&ss));
             if (self.is_valid()) nb::setattr(self, "_svc", nb::cast(svc));
         },
          nb::arg("svc"))
     .def("setStateValidityChecker",
         [](og::SimpleSetup &ss, const ompl::base::StateValidityCheckerPtr &svc) {
             ss.setStateValidityChecker(svc);
             nb::object self = nb::find(nb::cast(&ss));
             if (self.is_valid()) nb::setattr(self, "_svc", nb::cast(svc));
         },
          nb::arg("svc"))
     
     // setOptimizationObjective
     .def("setOptimizationObjective",
          &og::SimpleSetup::setOptimizationObjective,
          nb::arg("objective"))
     
     // setStartAndGoalStates, addStartState, etc.
     .def("setStartAndGoalStates",
          [](og::SimpleSetup &ss, const ob::State *start, const ob::State *goal, double threshold) {
               ss.getProblemDefinition()->setStartAndGoalStates(start, goal, threshold);
               ss.getProblemDefinition()->clearSolutionPaths();
               static_cast<SimpleSetupPublicist&>(ss).configured_ = false;
          },
          nb::arg("start"), nb::arg("goal"),
          nb::arg("threshold") = std::numeric_limits<double>::epsilon())
     
     .def("clearStartStates",
          &og::SimpleSetup::clearStartStates)

     .def("addStartState",
          [](og::SimpleSetup &ss, const ob::State * state) {
               ss.getProblemDefinition()->addStartState(state);
          },
          nb::arg("state"))

     .def("setStartState",
          [](og::SimpleSetup &ss, const ob::State * state) {
               ss.clearStartStates();
               ss.getProblemDefinition()->addStartState(state);
          },
          nb::arg("state"))

     .def("setGoalState",
          [](og::SimpleSetup &ss, const ob::State * state, double threshold) {
               ss.getProblemDefinition()->setGoalState(state, threshold);
               static_cast<SimpleSetupPublicist&>(ss).configured_ = false;
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
     .def("__repr__",
          [](const og::SimpleSetup &self) {
               std::ostringstream oss;
               self.print(oss);
               return oss.str();
          })
     
     // setup
     .def("setup",
          &og::SimpleSetup::setup);
}