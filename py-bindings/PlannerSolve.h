#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerTerminationCondition.h"

namespace ompl::binding
{
    namespace nb = nanobind;

    inline ompl::base::PlannerTerminationCondition toTerminationCondition(nb::handle what)
    {
        if (nb::isinstance<ompl::base::PlannerTerminationCondition>(what))
            return nb::cast<ompl::base::PlannerTerminationCondition>(what);
        if ((PyFloat_Check(what.ptr()) || PyLong_Check(what.ptr())) && !PyBool_Check(what.ptr()))
            return ompl::base::timedPlannerTerminationCondition(nb::cast<double>(what));
        if (PyCallable_Check(what.ptr()))
            return ompl::base::PlannerTerminationCondition(nb::cast<ompl::base::PlannerTerminationConditionFn>(what));
        throw nb::type_error("solve() expects a PlannerTerminationCondition, a callable or a time in seconds");
    }

    /// Planners block in solve() on threads of their own (PRM's solution checker, GoalLazySamples) that
    /// call back into Python, so solve() has to run with the GIL released.
    template <typename PlannerT>
    ompl::base::PlannerStatus solveWithoutGil(PlannerT &planner, nb::handle what)
    {
        const ompl::base::PlannerTerminationCondition ptc = toTerminationCondition(what);
        nb::gil_scoped_release release;
        return planner.solve(ptc);
    }

    template <typename PlannerT>
    auto solveBinding()
    {
        return [](PlannerT &planner, nb::handle what) { return solveWithoutGil(planner, what); };
    }

    /// A class's own "solve" hides every base overload, so planners re-bind Planner.solve(fn, checkInterval).
    template <typename PlannerT>
    auto solveFnBinding()
    {
        return [](PlannerT &planner, const ompl::base::PlannerTerminationConditionFn &fn, double checkInterval)
        {
            nb::gil_scoped_release release;
            return static_cast<ompl::base::Planner &>(planner).solve(fn, checkInterval);
        };
    }
}  // namespace ompl::binding
