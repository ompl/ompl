#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <memory>

#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"
#include "PyGC.h"

// Only the allocating space can free a control, and a control does not know its own size, so remember the space.
namespace ompl::binding::control
{
    namespace nb = nanobind;

    inline constexpr const char *controlSpaceKey = "_controlSpace";

    inline void tagControl(nb::handle control, nb::handle space)
    {
        if (gc::dictPtr(control.ptr()) != nullptr)
            nb::setattr(control, controlSpaceKey, space);
    }

    // nullptr for a control C++ handed to Python (a propagator argument, a path entry), whose size is unknown.
    inline const ompl::control::ControlSpace *taggedSpace(nb::handle control)
    {
        PyObject **dict = gc::dictPtr(control.ptr());
        if (dict == nullptr || *dict == nullptr)
            return nullptr;
        PyObject *space = PyDict_GetItemString(*dict, controlSpaceKey);
        const ompl::control::ControlSpace *result = nullptr;
        if (space == nullptr || !nb::try_cast(nb::handle(space), result))
            return nullptr;
        return result;
    }

    inline nb::object ownedControl(const ompl::control::ControlSpacePtr &space, ompl::control::Control *control)
    {
        std::shared_ptr<ompl::control::Control> owned(control,
                                                      [space](ompl::control::Control *c) { space->freeControl(c); });
        nb::object result = nb::cast(std::move(owned));
        tagControl(result, nb::cast(space));
        return result;
    }

    inline void checkControlIndex(unsigned int index, unsigned int size)
    {
        if (index >= size)
            throw nb::index_error("control index out of range");
    }
}  // namespace ompl::binding::control
