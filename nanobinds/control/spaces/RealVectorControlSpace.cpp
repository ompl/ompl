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
             nb::arg("space"))
        .def("sample", &oc::RealVectorControlUniformSampler::sample,
             nb::arg("control"));

    nb::class_<oc::RealVectorControlSpace::ControlType, oc::Control>(m, "RealVectorControlType")
        .def("__getitem__", [](const oc::RealVectorControlSpace::ControlType &ctrl, unsigned int i) {
            return ctrl[i];
        }, nb::rv_policy::reference_internal)
        .def("__setitem__", [](oc::RealVectorControlSpace::ControlType &ctrl, unsigned int i, double value) {
            ctrl[i] = value;
        });

    // 3) Bind the RealVectorControlSpace class
    nb::class_<oc::RealVectorControlSpace, oc::ControlSpace>(m, "RealVectorControlSpace")
        // Constructor that takes (StateSpacePtr, dimension)
        .def(nb::init<const ob::StateSpacePtr &, unsigned int>(),
             nb::arg("stateSpace"), nb::arg("dim"))

        // setBounds
        .def("setBounds", &oc::RealVectorControlSpace::setBounds,
             nb::arg("bounds"))

        // getBounds
        .def("getBounds", &oc::RealVectorControlSpace::getBounds,
             nb::rv_policy::reference_internal)

        // getDimension
        .def("getDimension", &oc::RealVectorControlSpace::getDimension)

        // copyControl
        .def("copyControl", &oc::RealVectorControlSpace::copyControl,
             nb::arg("destination"), nb::arg("source"))

        // equalControls
        .def("equalControls", &oc::RealVectorControlSpace::equalControls,
             nb::arg("control1"), nb::arg("control2"))

        // allocDefaultControlSampler
        .def("allocDefaultControlSampler", &oc::RealVectorControlSpace::allocDefaultControlSampler)

        // allocControl / freeControl
        .def("allocControl", &oc::RealVectorControlSpace::allocControl)

        .def("freeControl", &oc::RealVectorControlSpace::freeControl,
             nb::arg("control"))

        // nullControl
        .def("nullControl", &oc::RealVectorControlSpace::nullControl,
             nb::arg("control"))

        // printControl
        .def("printControl", [](const oc::RealVectorControlSpace &space, const oc::Control *ctrl) {
             space.printControl(ctrl, std::cout);
        }, nb::arg("control"))

        // getValueAddressAtIndex
        .def("getValueAddressAtIndex", &oc::RealVectorControlSpace::getValueAddressAtIndex,
             nb::arg("control"), nb::arg("index"))

        // printSettings
        .def("printSettings", [](const oc::RealVectorControlSpace &space) {
             space.printSettings(std::cout);
        })

        // setup
        .def("setup", &oc::RealVectorControlSpace::setup)

        // getSerializationLength, serialize, deserialize
        .def("getSerializationLength", &oc::RealVectorControlSpace::getSerializationLength)
        .def("serialize", &oc::RealVectorControlSpace::serialize,
             nb::arg("serialization"), nb::arg("ctrl"))
        .def("deserialize", &oc::RealVectorControlSpace::deserialize,
             nb::arg("ctrl"), nb::arg("serialization"));
}