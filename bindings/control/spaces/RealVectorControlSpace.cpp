#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"

#include "../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initSpaces_RealVectorControlSpace(nb::module_ &m)
{
    nb::class_<oc::RealVectorControlUniformSampler, oc::ControlSampler>(m, "RealVectorControlUniformSampler")
        .def(nb::init<const oc::ControlSpace *>(),
             nb::arg("space"),
             "Construct a RealVectorControlUniformSampler for a given control space")
        .def("sample", &oc::RealVectorControlUniformSampler::sample,
             nb::arg("control"),
             "Sample a random control uniformly within the bounds");

    nb::class_<oc::RealVectorControlSpace::ControlType, oc::Control>(m, "RealVectorControlType")
        .def("__getitem__", [](const oc::RealVectorControlSpace::ControlType &ctrl, unsigned int i) {
            return ctrl[i];
        }, "Get the i-th component of this control")
        .def("__setitem__", [](oc::RealVectorControlSpace::ControlType &ctrl, unsigned int i, double value) {
            ctrl[i] = value;
        }, "Set the i-th component of this control");

    // 3) Bind the RealVectorControlSpace class
    nb::class_<oc::RealVectorControlSpace, oc::ControlSpace>(m, "RealVectorControlSpace")
        // Constructor that takes (StateSpacePtr, dimension)
        .def(nb::init<const ob::StateSpacePtr &, unsigned int>(),
             nb::arg("stateSpace"), nb::arg("dim"),
             "Construct a RealVectorControlSpace for a given dimension")

        // setBounds
        .def("setBounds", &oc::RealVectorControlSpace::setBounds,
             nb::arg("bounds"),
             "Set the lower/upper bounds for each dimension of this control space")

        // getBounds
        .def("getBounds", &oc::RealVectorControlSpace::getBounds,
             nb::rv_policy::reference_internal,
             "Get the current bounds (RealVectorBounds) of this control space")

        // getDimension
        .def("getDimension", &oc::RealVectorControlSpace::getDimension,
             "Return the dimension of this control space")

        // copyControl
        .def("copyControl", &oc::RealVectorControlSpace::copyControl,
             nb::arg("destination"), nb::arg("source"),
             "Copy one control into another")

        // equalControls
        .def("equalControls", &oc::RealVectorControlSpace::equalControls,
             nb::arg("control1"), nb::arg("control2"),
             "Return whether two controls are equal")

        // allocDefaultControlSampler
        .def("allocDefaultControlSampler", &oc::RealVectorControlSpace::allocDefaultControlSampler,
             "Allocate the default control sampler for this space (uniform)")

        // allocControl / freeControl
        .def("allocControl", &oc::RealVectorControlSpace::allocControl,
             "Allocate memory for a control in this space")
        .def("freeControl", &oc::RealVectorControlSpace::freeControl,
             nb::arg("control"),
             "Free the memory for a control allocated by allocControl()")

        // nullControl
        .def("nullControl", &oc::RealVectorControlSpace::nullControl,
             nb::arg("control"),
             "Set the control to a 'null' value (commonly all zeros)")

        // printControl
        .def("printControl", [](const oc::RealVectorControlSpace &space, const oc::Control *ctrl) {
             space.printControl(ctrl, std::cout);
        }, nb::arg("control"),
        "Return a string describing the specified control")

        // getValueAddressAtIndex
        .def("getValueAddressAtIndex", &oc::RealVectorControlSpace::getValueAddressAtIndex,
             nb::arg("control"), nb::arg("index"),
             "Return a pointer to the i-th value in the given control")

        // printSettings
        .def("printSettings", [](const oc::RealVectorControlSpace &space) {
             space.printSettings(std::cout);
        }, "Return a string describing the control space settings")

        // setup
        .def("setup", &oc::RealVectorControlSpace::setup,
             "Setup this control space (usually checks bounds, etc.)")

        // getSerializationLength, serialize, deserialize
        .def("getSerializationLength", &oc::RealVectorControlSpace::getSerializationLength,
             "Return the number of bytes needed to serialize a control of this space")
        .def("serialize", &oc::RealVectorControlSpace::serialize,
             nb::arg("serialization"), nb::arg("ctrl"),
             "Serialize the given control into the provided memory buffer")
        .def("deserialize", &oc::RealVectorControlSpace::deserialize,
             nb::arg("ctrl"), nb::arg("serialization"),
             "Deserialize from the given memory buffer into the provided control");
}