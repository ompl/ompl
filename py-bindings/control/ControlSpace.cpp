#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/control/ControlSpace.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/Control.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_ControlSpace(nb::module_ &m)
{
    nb::class_<oc::ControlSpace>(m, "ControlSpace");

    nb::class_<oc::CompoundControlSpace, oc::ControlSpace>(m, "CompoundControlSpace")
        .def(nb::init<ob::StateSpacePtr>(), nb::arg("stateSpace"))
        .def("addSubspace", &oc::CompoundControlSpace::addSubspace, nb::arg("component"))
        .def("getSubspaceCount", &oc::CompoundControlSpace::getSubspaceCount)
        // Overload for getSubspace by index
        .def("getSubspace", nb::overload_cast<unsigned int>(&oc::CompoundControlSpace::getSubspace, nb::const_),
             nb::arg("index"), nb::rv_policy::reference_internal)
        // Overload for getSubspace by name
        .def("getSubspace", nb::overload_cast<const std::string &>(&oc::CompoundControlSpace::getSubspace, nb::const_),
             nb::arg("name"), nb::rv_policy::reference_internal)
        // The override for getDimension
        .def("getDimension", &oc::CompoundControlSpace::getDimension)
        // The override for allocControl/freeControl
        .def("allocControl", &oc::CompoundControlSpace::allocControl)
        .def("freeControl", &oc::CompoundControlSpace::freeControl, nb::arg("control"))
        .def("copyControl", &oc::CompoundControlSpace::copyControl, nb::arg("destination"), nb::arg("source"))
        .def("equalControls", &oc::CompoundControlSpace::equalControls, nb::arg("control1"), nb::arg("control2"))
        .def("nullControl", &oc::CompoundControlSpace::nullControl, nb::arg("control"))
        .def("allocDefaultControlSampler", &oc::CompoundControlSpace::allocDefaultControlSampler)
        .def("getValueAddressAtIndex", &oc::CompoundControlSpace::getValueAddressAtIndex, nb::arg("control"),
             nb::arg("index"))
        .def(
            "printControl", [](const oc::CompoundControlSpace &cs, const oc::Control *ctrl)
            { cs.printControl(ctrl, std::cout); }, nb::arg("control"))
        .def("printSettings", [](const oc::CompoundControlSpace &cs) { cs.printSettings(std::cout); })
        .def("setup", &oc::CompoundControlSpace::setup)
        .def("getSerializationLength", &oc::CompoundControlSpace::getSerializationLength)
        .def("serialize", &oc::CompoundControlSpace::serialize, nb::arg("serialization"), nb::arg("ctrl"))
        .def("deserialize", &oc::CompoundControlSpace::deserialize, nb::arg("ctrl"), nb::arg("serialization"))
        .def("isCompound", &oc::CompoundControlSpace::isCompound)
        .def("lock", &oc::CompoundControlSpace::lock);
}