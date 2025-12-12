#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/control/ControlSpace.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/Control.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_ControlSpace(nb::module_ &m)
{
    // TODO [oc::ControlSpace][TRAMPOLINE]
    nb::class_<oc::ControlSpace>(m, "ControlSpace");
        
    nb::class_<oc::CompoundControlSpace, oc::ControlSpace>(m, "CompoundControlSpace")
        .def(nb::init<ob::StateSpacePtr>(),
            nb::arg("stateSpace"),
            "Construct a CompoundControlSpace for the given StateSpace.")
        .def("addSubspace", &oc::CompoundControlSpace::addSubspace,
            nb::arg("component"),
            "Add a sub-control-space to this compound space. Must be done before lock().")
        .def("getSubspaceCount", &oc::CompoundControlSpace::getSubspaceCount,
            "Return the number of subspaces in this compound space.")
        // Overload for getSubspace by index
        .def("getSubspace",
            nb::overload_cast<unsigned int>(&oc::CompoundControlSpace::getSubspace, nb::const_),
            nb::arg("index"),
            nb::rv_policy::reference_internal,
            "Return the sub-control-space at the given index.")
        // Overload for getSubspace by name
        .def("getSubspace",
            nb::overload_cast<const std::string &>(&oc::CompoundControlSpace::getSubspace, nb::const_),
            nb::arg("name"),
            nb::rv_policy::reference_internal,
            "Return the sub-control-space with the given name (throws if not found).")
        // The override for getDimension
        .def("getDimension", &oc::CompoundControlSpace::getDimension,
            "Return the total dimension (sum of subspace dimensions).")
        // The override for allocControl/freeControl
        .def("allocControl", &oc::CompoundControlSpace::allocControl,
            "Allocate a compound control (holds controls for each subspace).")
        .def("freeControl", &oc::CompoundControlSpace::freeControl,
            nb::arg("control"),
            "Free a control allocated by this compound space.")
        .def("copyControl", &oc::CompoundControlSpace::copyControl,
            nb::arg("destination"), nb::arg("source"),
            "Copy one compound control to another.")
        .def("equalControls", &oc::CompoundControlSpace::equalControls,
            nb::arg("control1"), nb::arg("control2"),
            "Return true if two compound controls are identical.")
        .def("nullControl", &oc::CompoundControlSpace::nullControl,
            nb::arg("control"),
            "Set a compound control to 'null' in all subspaces.")
        .def("allocDefaultControlSampler", &oc::CompoundControlSpace::allocDefaultControlSampler,
            "Allocate the default sampler for each subspace, returning a compound sampler.")
        .def("getValueAddressAtIndex", &oc::CompoundControlSpace::getValueAddressAtIndex,
            nb::arg("control"), nb::arg("index"),
            "Return a pointer to the i-th scalar of the compound control, or nullptr if out of range.")
        .def("printControl", 
            [](const oc::CompoundControlSpace &cs, const oc::Control *ctrl) {
                cs.printControl(ctrl, std::cout);
            },
            nb::arg("control"),
            "Return a string describing the specified compound control.")
        .def("printSettings",
            [](const oc::CompoundControlSpace &cs) {
                cs.printSettings(std::cout);
            },
            "Return a string describing the subspaces and settings.")
        .def("setup", &oc::CompoundControlSpace::setup,
            "Finish setting up all subspaces and ensure everything is consistent.")
        .def("getSerializationLength", &oc::CompoundControlSpace::getSerializationLength,
            "Return how many bytes are needed to serialize a compound control.")
        .def("serialize", &oc::CompoundControlSpace::serialize,
            nb::arg("serialization"), nb::arg("ctrl"),
            "Serialize the compound control into a buffer.")
        .def("deserialize", &oc::CompoundControlSpace::deserialize,
            nb::arg("ctrl"), nb::arg("serialization"),
            "Deserialize a compound control from a buffer.")
        .def("isCompound", &oc::CompoundControlSpace::isCompound,
            "Always returns true for a CompoundControlSpace.")
        .def("lock", &oc::CompoundControlSpace::lock,
            "Lock the compound control space, preventing further subspace additions.");
}