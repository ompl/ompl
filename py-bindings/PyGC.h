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

    /// Moves the strong reference to \e fn into \e owner's __dict__ under \e key, so the owner's tp_traverse
    /// can report it and its tp_clear can drop it. The functor handed to OMPL should capture a borrowed
    /// nb::handle plus the returned object, which is empty on success and only non-empty in the unlikely case
    /// that \e owner has no Python instance to stash on -- there, owning the callable leaks but does not dangle.
    inline nb::object stash(nb::handle owner, const char *key, nb::handle fn)
    {
        if (!owner.is_valid())
            return nb::borrow(fn);

        nb::setattr(owner, key, fn);
        return nb::object();
    }
}  // namespace ompl::binding::gc
