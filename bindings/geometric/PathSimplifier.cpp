#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <limits>
#include "ompl/geometric/PathSimplifier.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_PathSimplifier(nb::module_& m)
{
    nb::class_<og::PathSimplifier>(m, "PathSimplifier")
        .def(nb::init<ob::SpaceInformationPtr,
                      const ob::GoalPtr&,
                      const ob::OptimizationObjectivePtr&>(),
             nb::arg("si"),
             nb::arg("goal") = nullptr,
             nb::arg("obj")  = nullptr)
        .def("reduceVertices", &og::PathSimplifier::reduceVertices,
             nb::arg("path"),
             nb::arg("maxSteps")      = 0u,
             nb::arg("maxEmptySteps") = 0u,
             nb::arg("rangeRatio")    = 0.33)
        .def("ropeShortcutPath", &og::PathSimplifier::ropeShortcutPath,
             nb::arg("path"),
             nb::arg("delta")                = 1.0,
             nb::arg("equivalenceTolerance") = 0.1)
        .def("partialShortcutPath", &og::PathSimplifier::partialShortcutPath,
             nb::arg("path"),
             nb::arg("maxSteps")      = 0u,
             nb::arg("maxEmptySteps") = 0u,
             nb::arg("rangeRatio")    = 0.33,
             nb::arg("snapToVertex")  = 0.005)
        .def("perturbPath", &og::PathSimplifier::perturbPath,
             nb::arg("path"),
             nb::arg("stepSize"),
             nb::arg("maxSteps")      = 0u,
             nb::arg("maxEmptySteps") = 0u,
             nb::arg("snapToVertex")  = 0.005)
        .def("collapseCloseVertices", &og::PathSimplifier::collapseCloseVertices,
             nb::arg("path"),
             nb::arg("maxSteps")      = 0u,
             nb::arg("maxEmptySteps") = 0u)
        .def("smoothBSpline", &og::PathSimplifier::smoothBSpline,
             nb::arg("path"),
             nb::arg("maxSteps")  = 5u,
             nb::arg("minChange") = std::numeric_limits<double>::epsilon())
        .def("simplifyMax", &og::PathSimplifier::simplifyMax,
             nb::arg("path"))
        .def("simplify",
             nb::overload_cast<og::PathGeometric&, double, bool>(
                 &og::PathSimplifier::simplify),
             nb::arg("path"),
             nb::arg("maxTime"),
             nb::arg("atLeastOnce") = true)
        .def("simplify",
             nb::overload_cast<og::PathGeometric&, const ob::PlannerTerminationCondition&, bool>(
                 &og::PathSimplifier::simplify),
             nb::arg("path"),
             nb::arg("ptc"),
             nb::arg("atLeastOnce") = true)
        .def("findBetterGoal",
             nb::overload_cast<og::PathGeometric&, double, unsigned int, double, double>(
                 &og::PathSimplifier::findBetterGoal),
             nb::arg("path"),
             nb::arg("maxTime"),
             nb::arg("samplingAttempts") = 10u,
             nb::arg("rangeRatio")        = 0.33,
             nb::arg("snapToVertex")      = 0.005)
        .def("findBetterGoal",
             nb::overload_cast<og::PathGeometric&, const ob::PlannerTerminationCondition&, unsigned int, double, double>(
                 &og::PathSimplifier::findBetterGoal),
             nb::arg("path"),
             nb::arg("ptc"),
             nb::arg("samplingAttempts") = 10u,
             nb::arg("rangeRatio")        = 0.33,
             nb::arg("snapToVertex")      = 0.005)
        .def("freeStates",
             (void (og::PathSimplifier::*)(bool)) &og::PathSimplifier::freeStates,
             nb::arg("flag"))
        .def("freeStates",
             (bool (og::PathSimplifier::*)() const) &og::PathSimplifier::freeStates);
}
