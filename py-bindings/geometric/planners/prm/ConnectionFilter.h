#pragma once

#include <nanobind/nanobind.h>

#include <type_traits>
#include <utility>

#include "ompl/base/Planner.h"
#include "ompl/base/State.h"
#include "PyGC.h"

namespace ompl::binding::geometric
{
    namespace nb = nanobind;
    namespace gc = ompl::binding::gc;

    /// Python gets the milestones' states: vertex descriptors mean nothing outside the planner (LazyPRM's: pointers).
    template <typename Vertex, typename StateMap>
    struct StateConnectionFilter
    {
        StateMap states;
        gc::SharedRef fn;

        bool operator()(const Vertex &a, const Vertex &b) const
        {
            const ompl::base::State *sa = states[a];
            const ompl::base::State *sb = states[b];
            return gc::call<bool>(fn, sa, sb);
        }
    };

    template <typename PlannerT, typename Access>
    struct ConnectionFilterBinding
    {
        using Vertex = typename PlannerT::Vertex;
        using StateMap = std::decay_t<decltype(Access::states(std::declval<PlannerT &>()))>;
        using Functor = StateConnectionFilter<Vertex, StateMap>;

        static void set(PlannerT &planner, nb::object fn)
        {
            if (fn.is_none())
                planner.setConnectionFilter([](const Vertex &, const Vertex &) { return true; });
            else
                planner.setConnectionFilter(Functor{Access::states(planner), gc::SharedRef(std::move(fn))});
        }

        // Planners have no __dict__ of their own; a Python subclass's dict is visited by CPython.
        static int traverse(PyObject *self, visitproc visit, void *arg)
        {
            if (int rc = ompl::binding::gc::traverseParent<ompl::base::Planner>(self, visit, arg))
                return rc;
            if (!nb::inst_ready(self) || !gc::pythonOwned(self))
                return 0;
            if (const auto *f = Access::filter(*nb::inst_ptr<PlannerT>(self)).template target<Functor>())
                if (nb::handle held = f->fn.exclusive(); held.is_valid())
                    Py_VISIT(held.ptr());
            return 0;
        }

        static int clear(PyObject *self)
        {
            ompl::binding::gc::clearParent<ompl::base::Planner>(self);
            if (nb::inst_ready(self) && gc::pythonOwned(self))
            {
                PlannerT &planner = *nb::inst_ptr<PlannerT>(self);
                if (Access::filter(planner).template target<Functor>() != nullptr)
                    planner.setConnectionFilter([](const Vertex &, const Vertex &) { return true; });
            }
            return 0;
        }

        static inline PyType_Slot slots[] = {{Py_tp_traverse, (void *)traverse},
                                             {Py_tp_clear, (void *)clear},
                                             {0, nullptr}};
    };
}  // namespace ompl::binding::geometric
