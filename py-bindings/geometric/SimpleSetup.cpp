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
#include "ompl/util/Exception.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct SimpleSetupPublicist : public og::SimpleSetup
{
    using og::SimpleSetup::configured_;
};

void ompl::binding::geometric::init_SimpleSetup(nb::module_ &m);

// The SI and pdef are shared with the planner, the simplifier and the user, so only what ss alone holds counts.
int simple_setup_geometric_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;
    if (int rc = gc::visitDict(self, visit, arg))
        return rc;
    if (!gc::pythonOwned(self))
        return 0;

    const auto &ss = *nb::inst_ptr<og::SimpleSetup>(self);
    if (int rc = gc::visitHeld(ss.getPlanner(), visit, arg))
        return rc;
    return gc::visitHeld(ss.getPlannerAllocator(), visit, arg);
}

int simple_setup_geometric_tp_clear(PyObject *self)
{
    if (nb::inst_ready(self) && gc::pythonOwned(self))
    {
        auto &ss = *nb::inst_ptr<og::SimpleSetup>(self);
        if (gc::holdsPython(ss.getPlannerAllocator()))
            ss.setPlannerAllocator(ob::PlannerAllocator());
        if (gc::holdsPython(ss.getPlanner()))
            ss.setPlanner(ob::PlannerPtr());
    }
    gc::clearDict(self);
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
          [](const og::SimpleSetup &ss) -> nb::object {
               const ob::PlannerAllocator &pa = ss.getPlannerAllocator();
               if (nb::handle fn = gc::callableOf(pa); fn.is_valid())
                    return nb::borrow(fn);
               return pa ? nb::cast(pa) : nb::none();
          })

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

     // Shares ownership: clear(), setStartAndGoalStates() and solve() drop the problem definition's path.
     .def("getSolutionPath",
          [](const og::SimpleSetup &self) {
               const ob::ProblemDefinitionPtr &pdef = self.getProblemDefinition();
               auto path = pdef ? std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath()) : nullptr;
               if (!path)
                    throw ompl::Exception("No solution path");
               return path;
          })

     // getPlannerData
     .def("getPlannerData",
          [](og::SimpleSetup &self, ob::PlannerData &pd) {
               self.getPlannerData(pd);
          },
          nb::arg("plannerData"))

     // setStateValidityChecker (two overloads)
     .def("setStateValidityChecker",
         [](og::SimpleSetup &ss, const ob::StateValidityCheckerPtr &svc) { ss.setStateValidityChecker(svc); },
          nb::arg("svc"))
     // The SpaceInformation holds the checker, so its binding wraps the callable.
     .def("setStateValidityChecker",
         [](og::SimpleSetup &ss, nb::callable svc) {
             nb::cast(ss.getSpaceInformation()).attr("setStateValidityChecker")(svc);
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
          [](og::SimpleSetup &ss, nb::callable allocator) {
               ss.setPlannerAllocator(gc::function<ob::PlannerAllocator>(std::move(allocator)));
          },
          nb::arg("allocator"))
     .def("setPlannerAllocator",
          [](og::SimpleSetup &ss, std::nullptr_t) { ss.setPlannerAllocator(ob::PlannerAllocator()); },
          nb::arg("allocator").none())

     // setup() keeps the GIL for Python callbacks that may not take it themselves; see PlannerSolve.h for solve.
     .def("solve",
          [](og::SimpleSetup &ss, double time) {
               ss.setup();
               nb::gil_scoped_release release;
               return ss.solve(time);
          },
          nb::arg("time") = 1.0)
     .def("solve",
          [](og::SimpleSetup &ss, const ob::PlannerTerminationCondition &ptc) {
               ss.setup();
               nb::gil_scoped_release release;
               return ss.solve(ptc);
          },
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
