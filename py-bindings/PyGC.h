#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

/// Helpers for keeping Python's garbage collector informed about references that C++ holds.
///
/// A C++ object that owns a Python reference forms a cycle Python cannot break on its own unless a
/// Py_tp_traverse slot reports that reference and a Py_tp_clear slot drops it, see
/// https://nanobind.readthedocs.io/en/latest/refleaks.html
///
///  * When the Python object that owns the callable is itself the bound type, give it gcSlots<> naming the
///    nb::object members.
///
///  * When the reference sits inside an OMPL object (a validity checker held by a SpaceInformation, a planner
///    allocator held by a SimpleSetup), OMPL owns it: a shared_ptr from nanobind's caster, a HoldsCallable
///    adapter, or a std::function made by function<>(). Never let OMPL borrow a reference stashed in __dict__:
///    the wrapper may be a view that dies first, and OMPL copies its pointers. The holder's tp_traverse reports
///    the reference with visitHeld(), only while the holder's wrapper is pythonOwned(), and its tp_clear
///    resets the members holdsPython() names. visitHeld() skips a reference that has more than one holder,
///    so each is reported at most once; a shared one leaks rather than dangles.
namespace ompl::binding::gc
{
    namespace nb = nanobind;

    inline PyObject **dictPtr(PyObject *self)
    {
        return _PyObject_GetDictPtr(self);
    }

    inline int visitDict(PyObject *self, visitproc visit, void *arg)
    {
        PyObject **dict = dictPtr(self);
        if (dict && *dict)
            Py_VISIT(*dict);
        return 0;
    }

    inline void clearDict(PyObject *self)
    {
        PyObject **dict = dictPtr(self);
        if (dict && *dict)
            Py_CLEAR(*dict);
    }

    /// True when Python owns the C++ instance behind the nanobind wrapper \e self, so the instance cannot
    /// outlive it. A wrapper returned from a C++ shared_ptr or by reference is only a view.
    inline bool pythonOwned(nb::handle self)
    {
        return self.is_valid() && nb::inst_state(self).second;
    }

    /// Drops \e obj under the GIL: OMPL may destroy the C++ owner on a thread that does not hold it.
    inline void releaseWithGil(nb::object &obj) noexcept
    {
        if (!obj.is_valid())
            return;
        if (!nb::is_alive())
        {
            obj.release();
            return;
        }
        nb::gil_scoped_acquire gil;
        obj.reset();
    }

    /// A strong reference that C++ may copy and destroy on any thread. Copies share the one reference.
    class SharedRef
    {
    public:
        SharedRef() = default;

        explicit SharedRef(nb::object obj) : holder_(std::make_shared<Holder>(std::move(obj)))
        {
        }

        nb::handle get() const
        {
            return holder_ ? nb::handle(holder_->obj) : nb::handle();
        }

        /// The reference while this is its only holder, otherwise empty.
        nb::handle exclusive() const
        {
            return holder_ && holder_.use_count() == 1 ? nb::handle(holder_->obj) : nb::handle();
        }

    private:
        struct Holder
        {
            explicit Holder(nb::object o) : obj(std::move(o))
            {
            }
            Holder(const Holder &) = delete;
            Holder &operator=(const Holder &) = delete;
            ~Holder()
            {
                releaseWithGil(obj);
            }

            nb::object obj;
        };

        std::shared_ptr<Holder> holder_;
    };

    /// Calls \e fn under the GIL and converts the result to \e R.
    template <typename R, typename... Args>
    R call(const SharedRef &fn, Args &&...args)
    {
        nb::gil_scoped_acquire gil;
        nb::handle callable = fn.get();
        if (!callable.is_valid())
            throw std::runtime_error("the Python callback is no longer set");
        nb::object result = callable(std::forward<Args>(args)...);
        if constexpr (!std::is_void_v<R>)
            return nb::cast<R>(result);
    }

    /// The functor function<>() stores in a std::function.
    template <typename Signature>
    struct PyFunction;

    template <typename R, typename... Args>
    struct PyFunction<R(Args...)>
    {
        SharedRef fn;

        R operator()(Args... args) const
        {
            return call<R>(fn, std::forward<Args>(args)...);
        }
    };

    template <typename F>
    struct PyFunctionFor;

    template <typename Signature>
    struct PyFunctionFor<std::function<Signature>>
    {
        using type = PyFunction<Signature>;
    };

    /// A std::function of type \e F (e.g. ob::PlannerAllocator) that owns \e fn and calls it under the GIL.
    template <typename F>
    F function(nb::object fn)
    {
        return F(typename PyFunctionFor<F>::type{SharedRef(std::move(fn))});
    }

    /// The Python callable behind a std::function made by function<>(), or an empty handle.
    template <typename Signature>
    nb::handle callableOf(const std::function<Signature> &f)
    {
        const auto *pf = f.template target<PyFunction<Signature>>();
        return pf ? pf->fn.get() : nb::handle();
    }

    /// Base for C++ adapters (a checker, a propagator) that own a Python callable on OMPL's behalf.
    struct HoldsCallable
    {
        explicit HoldsCallable(nb::object fn) : callable(std::move(fn))
        {
        }
        virtual ~HoldsCallable() = default;

        SharedRef callable;
    };

    /// Reports the Python reference \e ptr owns: a nanobind py_deleter reference, or a HoldsCallable's
    /// callable. Only while \e ptr is the sole owner of its control block, so each reference is reported once.
    template <typename T>
    int visitHeld(const std::shared_ptr<T> &ptr, visitproc visit, void *arg)
    {
        if (!ptr || ptr.use_count() != 1)
            return 0;
        nb::handle held;
        if (const auto *deleter = std::get_deleter<nb::detail::py_deleter>(ptr))
            held = deleter->o;
        if constexpr (std::is_polymorphic_v<T>)
            if (!held.is_valid())
                if (const auto *holder = dynamic_cast<const HoldsCallable *>(ptr.get()))
                    held = holder->callable.exclusive();
        if (held.is_valid())
            Py_VISIT(held.ptr());
        return 0;
    }

    template <typename Signature>
    int visitHeld(const std::function<Signature> &f, visitproc visit, void *arg)
    {
        if (const auto *pf = f.template target<PyFunction<Signature>>())
            if (nb::handle held = pf->fn.exclusive(); held.is_valid())
                Py_VISIT(held.ptr());
        return 0;
    }

    /// True when \e ptr keeps a Python object alive, whether or not visitHeld() would report it: the members a
    /// holder's tp_clear should reset.
    template <typename T>
    bool holdsPython(const std::shared_ptr<T> &ptr)
    {
        if (!ptr)
            return false;
        if (std::get_deleter<nb::detail::py_deleter>(ptr) != nullptr)
            return true;
        if constexpr (std::is_polymorphic_v<T>)
            return dynamic_cast<const HoldsCallable *>(ptr.get()) != nullptr;
        return false;
    }

    template <typename Signature>
    bool holdsPython(const std::function<Signature> &f)
    {
        return f.template target<PyFunction<Signature>>() != nullptr;
    }

    /// A type with its own tp_traverse replaces the one it would inherit from Parent, so it runs Parent's
    /// first; that one also visits the type, so the caller must not visit it again.
    template <typename Parent>
    int traverseParent(PyObject *self, visitproc visit, void *arg)
    {
        auto *parent = (PyTypeObject *)nb::type<Parent>().ptr();
        if (auto traverse = (traverseproc)PyType_GetSlot(parent, Py_tp_traverse))
            return traverse(self, visit, arg);
        Py_VISIT(Py_TYPE(self));
        return 0;
    }

    template <typename Parent>
    void clearParent(PyObject *self)
    {
        auto *parent = (PyTypeObject *)nb::type<Parent>().ptr();
        if (auto clear = (inquiry)PyType_GetSlot(parent, Py_tp_clear))
            clear(self);
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

    /// gcSlots<> for a type whose Parent has GC slots of its own, which it would otherwise replace.
    template <typename Parent, typename T, nb::object T::*...Members>
    int traverseOver(PyObject *self, visitproc visit, void *arg)
    {
        if (int rc = traverseParent<Parent>(self, visit, arg))
            return rc;
        if (!nb::inst_ready(self))
            return 0;

        T *obj = nb::inst_ptr<T>(self);
        for (const nb::object *member : {&(obj->*Members)...})
            if (member->is_valid())
                if (int rc = visit(member->ptr(), arg); rc != 0)
                    return rc;
        return 0;
    }

    template <typename Parent, typename T, nb::object T::*...Members>
    int clearOver(PyObject *self)
    {
        clearParent<Parent>(self);
        return clear<T, Members...>(self);
    }

    template <typename Parent, typename T, nb::object T::*...Members>
    PyType_Slot gcSlotsOver[] = {{Py_tp_traverse, (void *)traverseOver<Parent, T, Members...>},
                                 {Py_tp_clear, (void *)clearOver<Parent, T, Members...>},
                                 {0, 0}};
}  // namespace ompl::binding::gc
