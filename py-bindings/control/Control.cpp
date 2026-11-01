#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"
#include "OwnedControl.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;

namespace
{
    const oc::CompoundControlSpace *compoundSpaceOf(nb::handle control)
    {
        return dynamic_cast<const oc::CompoundControlSpace *>(ompl::binding::control::taggedSpace(control));
    }

    nb::object component(nb::pointer_and_handle<oc::CompoundControl> cc, unsigned int idx,
                         const oc::CompoundControlSpace *space)
    {
        nb::object sub = nb::cast((*cc.p)[idx], nb::rv_policy::reference_internal, cc.h);
        if (space != nullptr)
            ompl::binding::control::tagControl(sub, nb::cast(space->getSubspace(idx)));
        return sub;
    }
}  // namespace

void ompl::binding::control::init_Control(nb::module_ &m)
{
    nb::class_<ompl::control::Control>(m, "Control");
    nb::class_<ompl::control::CompoundControl, ompl::control::Control>(m, "CompoundControl", nb::dynamic_attr())
        .def(nb::init<>())
        .def(
            "__getitem__",
            [](nb::pointer_and_handle<oc::CompoundControl> cc, unsigned int idx)
            {
                const oc::CompoundControlSpace *space = compoundSpaceOf(cc.h);
                if (space != nullptr)
                    checkControlIndex(idx, space->getSubspaceCount());
                return component(cc, idx, space);
            },
            nb::arg("index"), "Return the i-th sub-control in this compound control.")
        .def("__iter__",
             [](nb::pointer_and_handle<oc::CompoundControl> cc)
             {
                 const oc::CompoundControlSpace *space = compoundSpaceOf(cc.h);
                 if (space == nullptr)
                     throw nb::type_error("the size of this control is unknown; index it with control[i]");
                 nb::list components;
                 for (unsigned int i = 0; i < space->getSubspaceCount(); ++i)
                     components.append(component(cc, i, space));
                 return nb::iter(components);
             });
    // TODO: Expose the components member variable
}
