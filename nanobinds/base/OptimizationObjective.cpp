#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include "ompl/base/OptimizationObjective.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

void ompl::binding::base::init_OptimizationObjective(nb::module_ &m)
{
    struct PyOptimizationObjective : ob::OptimizationObjective
    {
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

        // Optional override
        void print(std::ostream &out) const override
        {
            NB_OVERRIDE(print, out);
        }
    };

    nb::class_<ob::OptimizationObjective, PyOptimizationObjective /* <-- trampoline */>(m, "OptimizationObjective")
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
        .def("setCostToGoHeuristic", &ob::OptimizationObjective::setCostToGoHeuristic, nb::arg("costToGoFn"))
        .def("__str__",
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
}
