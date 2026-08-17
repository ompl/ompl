#pragma once

#include <nanobind/nanobind.h>

/// Helpers for keeping Python's garbage collector informed about references that C++ holds.
///
/// A C++ object that owns an nb::object forms a cycle Python cannot break on its own: the collector
/// cannot see the reference, so the cycle is never proven unreachable. nanobind's remedy is a
/// Py_tp_traverse slot that reports the reference and a Py_tp_clear slot that drops it, see
/// https://nanobind.readthedocs.io/en/latest/refleaks.html
///
/// Two shapes show up in these bindings:
///
///  * The Python object that owns the callable is itself the bound type. Give it gcSlots<> naming the
///    nb::object members and the cycle becomes collectable with no further work.
///
///  * The callable is handed to an OMPL class we cannot add members to (a validity checker, a state
///    propagator). There the binding stores the sole strong reference in the owning instance's
///    __dict__ and lets OMPL hold a borrowing functor, so tp_clear releases it by clearing the dict.
///    Handing OMPL a reference-owning std::function instead would reintroduce the invisible reference:
///    nanobind's std::function caster keeps the callable alive with a bare Py_INCREF.
namespace ompl::binding::gc
{
    namespace nb = nanobind;

    inline PyObject **dictPtr(PyObject *self)
    {
        return _PyObject_GetDictPtr(self);
    }

    /// Reports the nb::object members of a bound instance to the collector.
    template <typename T, nb::object T::*...Members>
    int traverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        // Traversal can happen before the C++ constructor has run.
        if (!nb::inst_ready(self))
            return 0;

        T *obj = nb::inst_ptr<T>(self);
        for (const nb::object *member : {&(obj->*Members)...})
            if (member->is_valid())
                if (int rc = visit(member->ptr(), arg); rc != 0)
                    return rc;
        return 0;
    }

    /// Drops those references, breaking the cycle.
    template <typename T, nb::object T::*...Members>
    int clear(PyObject *self)
    {
        if (!nb::inst_ready(self))
            return 0;

        T *obj = nb::inst_ptr<T>(self);
        for (nb::object *member : {&(obj->*Members)...})
            member->reset();
        return 0;
    }

    /// Pass to nb::type_slots() when binding T, naming every nb::object member T owns.
    template <typename T, nb::object T::*...Members>
    PyType_Slot gcSlots[] = {{Py_tp_traverse, (void *)traverse<T, Members...>},
                             {Py_tp_clear, (void *)clear<T, Members...>},
                             {0, 0}};

    /// The reference C++ has to keep for itself: empty when \e owner can hold it in __dict__ instead, where
    /// tp_traverse reports it and tp_clear drops it. Non-empty only when \e owner has no Python instance to
    /// stash on, where owning the callable leaks but does not dangle.
    ///
    /// Capture the result alongside a borrowed nb::handle in the functor handed to OMPL, install that functor,
    /// and only then write \e fn to __dict__. Publishing first would drop the previous callback while OMPL
    /// still borrows it, and a __del__ that re-enters the binding would read freed memory.
    inline nb::object keeper(nb::handle owner, nb::handle fn)
    {
        return owner.is_valid() ? nb::object() : nb::borrow(fn);
    }

    /// Hands \e obj to OMPL through \e set without letting C++ own the only reference to its Python wrapper:
    /// \e owner keeps that wrapper in __dict__ under \e key, where tp_traverse reports it and tp_clear drops
    /// it, and OMPL gets a non-owning alias. With no owner to stash on, C++ owns the wrapper instead, which
    /// leaks but does not dangle. With no wrapper at all \e obj is pure C++ and there is nothing to keep alive.
    template <typename Ptr, typename T, typename Setter>
    void installBorrowed(nb::handle owner, const char *key, T *obj, Setter set)
    {
        nb::object wrapper = nb::find(*obj);
        if (owner.is_valid() && wrapper.is_valid())
        {
            set(Ptr(obj, [](T *) {}));
            nb::setattr(owner, key, wrapper);
        }
        else if (wrapper.is_valid())
            set(nb::cast<Ptr>(wrapper));
        else
            set(Ptr(obj, [](T *) {}));
    }
}  // namespace ompl::binding::gc
