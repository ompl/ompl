#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/control/SpaceInformation.h"
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/Control.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/base/SpaceInformation.h"  // ensure the base class is included
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

void ompl::binding::control::init_SpaceInformation(nb::module_ &m)
{
    nb::class_<oc::SpaceInformation, ob::SpaceInformation>(m, "SpaceInformation")
        //
        // Constructor
        //
        .def(nb::init<ob::StateSpacePtr, oc::ControlSpacePtr>(), nb::arg("stateSpace"), nb::arg("controlSpace"),
             "Construct a control-based SpaceInformation from a given state space and control space.")

        // getControlSpace
        .def("getControlSpace", &oc::SpaceInformation::getControlSpace, nb::rv_policy::reference_internal,
             "Return the ControlSpace associated with this SpaceInformation.")

        // allocControl, freeControl, copyControl, cloneControl
        .def("allocControl", &oc::SpaceInformation::allocControl, "Allocate a control in this control space.")
        .def("freeControl", &oc::SpaceInformation::freeControl, nb::arg("control"),
             "Free a control allocated by allocControl.")
        .def("copyControl", &oc::SpaceInformation::copyControl, nb::arg("destination"), nb::arg("source"),
             "Copy one control into another.")
        .def("cloneControl", &oc::SpaceInformation::cloneControl, nb::arg("source"),
             "Allocate a new control and copy the specified source into it.")

        // printControl
        .def(
            "printControl",
            [](const oc::SpaceInformation &si, const oc::Control *ctrl) { si.printControl(ctrl, std::cout); },
            nb::arg("control"), "Return a string representation of the specified control.")

        // equalControls, nullControl
        .def("equalControls", &oc::SpaceInformation::equalControls, nb::arg("control1"), nb::arg("control2"),
             "Return whether two controls are equal.")
        .def("nullControl", &oc::SpaceInformation::nullControl, nb::arg("control"),
             "Set the control to a 'null' value (commonly all zeros).")

        // allocControlSampler
        .def("allocControlSampler", &oc::SpaceInformation::allocControlSampler,
             "Allocate a control sampler. Either default or from a user-specified allocator.")

        // setMinMaxControlDuration, etc.
        .def("setMinMaxControlDuration", &oc::SpaceInformation::setMinMaxControlDuration, nb::arg("minSteps"),
             nb::arg("maxSteps"), "Set the minimum and maximum control duration (in propagation steps).")
        .def("setMinControlDuration", &oc::SpaceInformation::setMinControlDuration, nb::arg("minSteps"),
             "Set the minimum control duration (in propagation steps).")
        .def("setMaxControlDuration", &oc::SpaceInformation::setMaxControlDuration, nb::arg("maxSteps"),
             "Set the maximum control duration (in propagation steps).")
        .def("getMinControlDuration", &oc::SpaceInformation::getMinControlDuration,
             "Return the minimum control duration (in steps).")
        .def("getMaxControlDuration", &oc::SpaceInformation::getMaxControlDuration,
             "Return the maximum control duration (in steps).")

        // Directed control sampler
        .def("allocDirectedControlSampler", &oc::SpaceInformation::allocDirectedControlSampler,
             "Allocate a directed control sampler. May default to a fallback if no custom allocator is set.")
        .def("setDirectedControlSamplerAllocator", &oc::SpaceInformation::setDirectedControlSamplerAllocator,
             nb::arg("dcsa"), "Set a function/functor to allocate directed control samplers.")
        .def("clearDirectedSamplerAllocator", &oc::SpaceInformation::clearDirectedSamplerAllocator,
             "Clear any custom directed sampler allocator, revert to default.")

        // StatePropagator
        .def("getStatePropagator", &oc::SpaceInformation::getStatePropagator, nb::rv_policy::reference_internal,
             "Return the state propagator.")

        // setStatePropagator
        .def("setStatePropagator",
            nb::overload_cast<const ompl::control::StatePropagatorFn&>(&ompl::control::SpaceInformation::setStatePropagator),
             nb::arg("sp"), "Set the state propagator from a StatePropagatorFn.")

        // setPropagationStepSize, getPropagationStepSize
        .def("setPropagationStepSize", &oc::SpaceInformation::setPropagationStepSize, nb::arg("stepSize"),
             "Set the maximum step size to use when discretizing controls.")
        .def("getPropagationStepSize", &oc::SpaceInformation::getPropagationStepSize,
             "Return the maximum step size used when discretizing controls.")

        // propagate(...) overloads
        .def(
            "propagate",
            [](const oc::SpaceInformation &si, const ob::State *state, const oc::Control *control, int steps,
               ob::State *result) { si.propagate(state, control, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"),
            "Propagate forward from 'start' for 'steps' discrete steps, writing into 'result'.")
        .def("canPropagateBackward", &oc::SpaceInformation::canPropagateBackward,
             "Return true if backward propagation is supported.")
        .def(
            "propagateWhileValid",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps,
               ob::State *result) { return si.propagateWhileValid(start, ctrl, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"),
            "Propagate 'steps' times while states remain valid, return how many steps were valid.")
        // overload that writes to a vector<State*>...
        .def(
            "propagateWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps, bool alloc)
            {
                // We'll wrap the version that writes to a std::vector<base::State*>.
                std::vector<ob::State *> path;
                si.propagate(start, ctrl, steps, path, alloc);
                return path;  // returns a Python list of pointers
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("alloc") = true,
            "Propagate for 'steps' steps, returning the intermediate states as a list.")
        .def(
            "propagateWhileValidWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps, bool alloc)
            {
                std::vector<ob::State *> path;
                unsigned int nValid = si.propagateWhileValid(start, ctrl, steps, path, alloc);
                return std::make_pair(nValid, path);
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("alloc") = true,
            "Propagate up to 'steps' times while states remain valid. "
            "Return (numberOfValidSteps, [listOfStates]).")

        // printSettings override
        .def(
            "printSettings", [](const oc::SpaceInformation &si) { si.printSettings(std::cout); },
            "Return a string describing the control-based space information.")

        // setup override
        .def("setup", &oc::SpaceInformation::setup,
             "Perform final checks and setup for the control-based space information.");
}