#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <memory>
#include <utility>

#include "PyGC.h"

// traverseOwner() reports the owner but nothing clears it: dropping it before the borrower would dangle.
namespace ompl::binding
{
    namespace nb = nanobind;

    struct OwnerRef
    {
        std::shared_ptr<const void> owner;
    };

    // Virtual so it is destroyed after T, whose destructor may use the borrowed pointer; T stays at offset 0.
    template <typename T>
    struct WithOwner : T, virtual OwnerRef
    {
        template <typename... Args>
        explicit WithOwner(std::shared_ptr<const void> o, Args &&...args)
          : OwnerRef{std::move(o)}, T(std::forward<Args>(args)...)
        {
        }
    };

    template <typename T, typename Impl, typename Owner, typename... Args>
    auto initWithOwnerAs()
    {
        return [](T *self, const std::shared_ptr<Owner> &owner, Args... args)
        { new (self) WithOwner<Impl>(owner, owner.get(), std::move(args)...); };
    }

    template <typename T, typename Owner, typename... Args>
    auto initWithOwner()
    {
        return initWithOwnerAs<T, T, Owner, Args...>();
    }

    template <typename T>
    int traverseOwner(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        if (const auto *ref = dynamic_cast<const OwnerRef *>(nb::inst_ptr<T>(self)))
            return gc::visitHeld(ref->owner, visit, arg);
        return 0;
    }

    template <typename T>
    PyType_Slot ownerSlots[] = {{Py_tp_traverse, (void *)traverseOwner<T>}, {0, 0}};
}  // namespace ompl::binding
