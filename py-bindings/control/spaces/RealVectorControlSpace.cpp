#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"

#include "../OwnedControl.h"
#include "../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

namespace
{
    using RVControl = oc::RealVectorControlSpace::ControlType;

    void checkIndex(nb::handle control, unsigned int i)
    {
        if (const oc::ControlSpace *space = ompl::binding::control::taggedSpace(control))
            ompl::binding::control::checkControlIndex(i, space->getDimension());
    }
}  // namespace

void ompl::binding::control::initSpaces_RealVectorControlSpace(nb::module_ &m)
{
    nb::class_<oc::RealVectorControlUniformSampler, oc::ControlSampler>(m, "RealVectorControlUniformSampler")
        .def(nb::init<const oc::ControlSpace *>(), nb::arg("space"), nb::keep_alive<1, 2>());

    nb::class_<RVControl, oc::Control>(m, "RealVectorControlType", nb::dynamic_attr())
        .def(
            "__getitem__",
            [](nb::pointer_and_handle<RVControl> ctrl, unsigned int i)
            {
                checkIndex(ctrl.h, i);
                return (*ctrl.p)[i];
            },
            nb::arg("index"))
        .def(
            "__setitem__",
            [](nb::pointer_and_handle<RVControl> ctrl, unsigned int i, double value)
            {
                checkIndex(ctrl.h, i);
                (*ctrl.p)[i] = value;
            },
            nb::arg("index"), nb::arg("value"))
        .def("__iter__",
             [](nb::pointer_and_handle<RVControl> ctrl)
             {
                 const oc::ControlSpace *space = taggedSpace(ctrl.h);
                 if (space == nullptr)
                     throw nb::type_error("the size of this control is unknown; index it with control[i]");
                 nb::list values;
                 for (unsigned int i = 0; i < space->getDimension(); ++i)
                     values.append(ctrl.p->values[i]);
                 return nb::iter(values);
             });

    // 3) Bind the RealVectorControlSpace class
    nb::class_<oc::RealVectorControlSpace, oc::ControlSpace>(m, "RealVectorControlSpace")
        // Constructor that takes (StateSpacePtr, dimension)
        .def(nb::init<const ob::StateSpacePtr &, unsigned int>(), nb::arg("stateSpace"), nb::arg("dim"))

        // setBounds
        .def("setBounds", &oc::RealVectorControlSpace::setBounds, nb::arg("bounds"))

        // getBounds
        .def("getBounds", &oc::RealVectorControlSpace::getBounds, nb::rv_policy::reference_internal)

        // getDimension
        .def("getDimension", &oc::RealVectorControlSpace::getDimension)

        // copyControl
        .def("copyControl", &oc::RealVectorControlSpace::copyControl, nb::arg("destination"), nb::arg("source"))

        // equalControls
        .def("equalControls", &oc::RealVectorControlSpace::equalControls, nb::arg("control1"), nb::arg("control2"))

        // allocDefaultControlSampler
        .def("allocDefaultControlSampler", &oc::RealVectorControlSpace::allocDefaultControlSampler)

        .def("allocControl",
             [](nb::pointer_and_handle<oc::RealVectorControlSpace> self)
             {
                 auto space = nb::cast<oc::ControlSpacePtr>(self.h);
                 return ownedControl(space, space->allocControl());
             })

        // nullControl
        .def("nullControl", &oc::RealVectorControlSpace::nullControl, nb::arg("control"))

        // printControl
        .def(
            "printControl", [](const oc::RealVectorControlSpace &space, const oc::Control *ctrl)
            { space.printControl(ctrl, std::cout); }, nb::arg("control"))

        // getValueAddressAtIndex
        .def("getValueAddressAtIndex", &oc::RealVectorControlSpace::getValueAddressAtIndex, nb::arg("control"),
             nb::arg("index"))

        // printSettings
        .def("printSettings", [](const oc::RealVectorControlSpace &space) { space.printSettings(std::cout); })

        // setup
        .def("setup", &oc::RealVectorControlSpace::setup)

        // getSerializationLength, serialize, deserialize
        .def("getSerializationLength", &oc::RealVectorControlSpace::getSerializationLength)
        .def("serialize", &oc::RealVectorControlSpace::serialize, nb::arg("serialization"), nb::arg("ctrl"))
        .def("deserialize", &oc::RealVectorControlSpace::deserialize, nb::arg("ctrl"), nb::arg("serialization"));
}
