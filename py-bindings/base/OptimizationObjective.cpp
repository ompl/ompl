#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include <sstream>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalRegion.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    // The heuristic is protected and has no getter.
    struct ObjectiveAccess : ob::OptimizationObjective
    {
        static const ob::CostToGoHeuristic &costToGo(const ob::OptimizationObjective &objective)
        {
            return objective.*(&ObjectiveAccess::costToGoFn_);
        }
    };

    int objectiveTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        const auto &objective = *nb::inst_ptr<ob::OptimizationObjective>(self);
        if (int rc = gc::visitHeld(objective.getSpaceInformation(), visit, arg))
            return rc;
        return gc::visitHeld(ObjectiveAccess::costToGo(objective), visit, arg);
    }

    int objectiveClear(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        auto &objective = *nb::inst_ptr<ob::OptimizationObjective>(self);
        if (gc::holdsPython(ObjectiveAccess::costToGo(objective)))
            objective.setCostToGoHeuristic(ob::CostToGoHeuristic());
        return 0;
    }

    const PyType_Slot objectiveSlots[] = {{Py_tp_traverse, (void *)objectiveTraverse},
                                          {Py_tp_clear, (void *)objectiveClear},
                                          {0, 0}};
}  // namespace

void ompl::binding::base::init_OptimizationObjective(nb::module_ &m)
{
    struct PyOptimizationObjective : ob::OptimizationObjective
    {
        // print() is not routed through the trampoline: std::ostream has no caster, so a Python override could
        // never be called. Python subclasses customise their text via __repr__ instead.
        NB_TRAMPOLINE(ob::OptimizationObjective, 19);
        // Optional override
        bool isSatisfied(ob::Cost c) const override
        {
            NB_OVERRIDE(isSatisfied, c);
        }

        // Optional override
        bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const override
        {
            NB_OVERRIDE(isCostBetterThan, c1, c2);
        }

        // Optional override
        bool isCostEquivalentTo(ob::Cost c1, ob::Cost c2) const override
        {
            NB_OVERRIDE(isCostEquivalentTo, c1, c2);
        }

        // Optional override
        bool isFinite(ob::Cost c) const override
        {
            NB_OVERRIDE(isFinite, c);
        }

        // Optional override
        ob::Cost betterCost(ob::Cost c1, ob::Cost c2) const override
        {
            NB_OVERRIDE(betterCost, c1, c2);
        }

        // Pure virtual function
        ob::Cost stateCost(const ob::State *s) const override
        {
            NB_OVERRIDE_PURE(stateCost, s);
        }

        // Pure virtual function
        ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE_PURE(motionCost, s1, s2);
        }

        // Optional override
        ob::Cost controlCost(const oc::Control *c, unsigned int steps) const override
        {
            NB_OVERRIDE(controlCost, c, steps);
        }

        // Optional override
        ob::Cost controlMotionCost(const ob::State *s1, const oc::Control *c, unsigned int steps,
                                   const ob::State *s2) const override
        {
            NB_OVERRIDE(controlMotionCost, s1, c, steps, s2);
        }

        // Optional override
        ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const override
        {
            NB_OVERRIDE(combineCosts, c1, c2);
        }

        // Optional override
        ob::Cost subtractCosts(ob::Cost c1, ob::Cost c2) const override
        {
            NB_OVERRIDE(subtractCosts, c1, c2);
        }

        // Optional override
        ob::Cost identityCost() const override
        {
            NB_OVERRIDE(identityCost);
        }

        // Optional override
        ob::Cost infiniteCost() const override
        {
            NB_OVERRIDE(infiniteCost);
        }

        // Optional override
        ob::Cost initialCost(const ob::State *s) const override
        {
            NB_OVERRIDE(initialCost, s);
        }

        // Optional override
        ob::Cost terminalCost(const ob::State *s) const override
        {
            NB_OVERRIDE(terminalCost, s);
        }

        // Optional override
        bool isSymmetric() const override
        {
            NB_OVERRIDE(isSymmetric);
        }

        // Optional override
        ob::Cost averageStateCost(unsigned int numStates) const override
        {
            NB_OVERRIDE(averageStateCost, numStates);
        }

        // Optional override
        ob::Cost motionCostHeuristic(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCostHeuristic, s1, s2);
        }

        // Optional override
        ob::Cost motionCostBestEstimate(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCostBestEstimate, s1, s2);
        }
    };

    nb::class_<ob::OptimizationObjective, PyOptimizationObjective /* <-- trampoline */>(m, "OptimizationObjective",
                                                                                        nb::type_slots(objectiveSlots))
        // constructor
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        // getters
        .def("getDescription", &ob::OptimizationObjective::getDescription)
        .def("getCostThreshold", &ob::OptimizationObjective::getCostThreshold)
        .def("hasCostToGoHeuristic", &ob::OptimizationObjective::hasCostToGoHeuristic)
        .def("costToGo", &ob::OptimizationObjective::costToGo, nb::arg("state"), nb::arg("goal"))
        .def("getSpaceInformation", &ob::OptimizationObjective::getSpaceInformation, nb::rv_policy::reference_internal)
        // setters
        .def("setCostThreshold", &ob::OptimizationObjective::setCostThreshold, nb::arg("cost"))
        .def(
            "setCostToGoHeuristic", [](ob::OptimizationObjective &objective, nb::callable costToGoFn)
            { objective.setCostToGoHeuristic(gc::function<ob::CostToGoHeuristic>(costToGoFn)); }, nb::arg("costToGoFn"))
        .def(
            "setCostToGoHeuristic", [](ob::OptimizationObjective &objective, std::nullptr_t)
            { objective.setCostToGoHeuristic(ob::CostToGoHeuristic()); }, nb::arg("costToGoFn").none())
        // virtual methods
        .def("isSatisfied", &ob::OptimizationObjective::isSatisfied, nb::arg("cost"))
        .def("isCostBetterThan", &ob::OptimizationObjective::isCostBetterThan, nb::arg("c1"), nb::arg("c2"))
        .def("isCostEquivalentTo", &ob::OptimizationObjective::isCostEquivalentTo, nb::arg("c1"), nb::arg("c2"))
        .def("isFinite", &ob::OptimizationObjective::isFinite, nb::arg("cost"))
        .def("betterCost", &ob::OptimizationObjective::betterCost, nb::arg("c1"), nb::arg("c2"))
        .def("stateCost", &ob::OptimizationObjective::stateCost, nb::arg("state"))
        .def("motionCost", &ob::OptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("controlCost", &ob::OptimizationObjective::controlCost, nb::arg("control"), nb::arg("steps"))
        .def("combineCosts", &ob::OptimizationObjective::combineCosts, nb::arg("c1"), nb::arg("c2"))
        .def("subtractCosts", &ob::OptimizationObjective::subtractCosts, nb::arg("c1"), nb::arg("c2"))
        .def("identityCost", &ob::OptimizationObjective::identityCost)
        .def("infiniteCost", &ob::OptimizationObjective::infiniteCost)
        .def("initialCost", &ob::OptimizationObjective::initialCost, nb::arg("state"))
        .def("terminalCost", &ob::OptimizationObjective::terminalCost, nb::arg("state"))
        .def("isSymmetric", &ob::OptimizationObjective::isSymmetric)
        .def("averageStateCost", &ob::OptimizationObjective::averageStateCost, nb::arg("numStates"))
        .def("motionCostHeuristic", &ob::OptimizationObjective::motionCostHeuristic, nb::arg("s1"), nb::arg("s2"))
        .def("motionCostBestEstimate", &ob::OptimizationObjective::motionCostBestEstimate, nb::arg("s1"), nb::arg("s2"))
        .def("__repr__",
             [](const ob::OptimizationObjective &obj)
             {
                 std::ostringstream oss;
                 obj.print(oss);
                 return oss.str();
             });

    // MultiOptimizationObjective - combines multiple objectives with weights
    nb::class_<ob::MultiOptimizationObjective, ob::OptimizationObjective>(m, "MultiOptimizationObjective")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("addObjective", &ob::MultiOptimizationObjective::addObjective, nb::arg("objective"),
             nb::arg("weight") = 1.0)
        .def("getObjectiveCount", &ob::MultiOptimizationObjective::getObjectiveCount)
        .def("getObjective", &ob::MultiOptimizationObjective::getObjective, nb::arg("idx"),
             nb::rv_policy::reference_internal)
        .def("getObjectiveWeight", &ob::MultiOptimizationObjective::getObjectiveWeight, nb::arg("idx"))
        .def("setObjectiveWeight", &ob::MultiOptimizationObjective::setObjectiveWeight, nb::arg("idx"),
             nb::arg("weight"))
        .def("lock", &ob::MultiOptimizationObjective::lock)
        .def("isLocked", &ob::MultiOptimizationObjective::isLocked);

    m.def(
        "goalRegionCostToGo",
        [](const ob::State *state, const ob::Goal *goal)
        {
            // OMPL static_casts the goal to a GoalRegion.
            if (dynamic_cast<const ob::GoalRegion *>(goal) == nullptr)
                throw nb::type_error("goalRegionCostToGo() needs a GoalRegion");
            return ob::goalRegionCostToGo(state, goal);
        },
        nb::arg("state"), nb::arg("goal"));
}
