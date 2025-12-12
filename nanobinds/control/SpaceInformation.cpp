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
        .def(nb::init<ob::StateSpacePtr, oc::ControlSpacePtr>(), nb::arg("stateSpace"), nb::arg("controlSpace"))

        // getControlSpace
        .def("getControlSpace", &oc::SpaceInformation::getControlSpace, nb::rv_policy::reference_internal)

        // allocControl, freeControl, copyControl, cloneControl
        .def("allocControl", &oc::SpaceInformation::allocControl)
        .def("freeControl", &oc::SpaceInformation::freeControl, nb::arg("control"))
        .def("copyControl", &oc::SpaceInformation::copyControl, nb::arg("destination"), nb::arg("source"))
        .def("cloneControl", &oc::SpaceInformation::cloneControl, nb::arg("source"))

        // printControl
        .def(
            "printControl",
            [](const oc::SpaceInformation &si, const oc::Control *ctrl) { si.printControl(ctrl, std::cout); },
            nb::arg("control"))

        // equalControls, nullControl
        .def("equalControls", &oc::SpaceInformation::equalControls, nb::arg("control1"), nb::arg("control2"))
        .def("nullControl", &oc::SpaceInformation::nullControl, nb::arg("control"))

        // allocControlSampler
        .def("allocControlSampler", &oc::SpaceInformation::allocControlSampler)

        // setMinMaxControlDuration, etc.
        .def("setMinMaxControlDuration", &oc::SpaceInformation::setMinMaxControlDuration, nb::arg("minSteps"), nb::arg("maxSteps"))
        .def("setMinControlDuration", &oc::SpaceInformation::setMinControlDuration, nb::arg("minSteps"))
        .def("setMaxControlDuration", &oc::SpaceInformation::setMaxControlDuration, nb::arg("maxSteps"))
        .def("getMinControlDuration", &oc::SpaceInformation::getMinControlDuration)
        .def("getMaxControlDuration", &oc::SpaceInformation::getMaxControlDuration)

        // Directed control sampler
        .def("allocDirectedControlSampler", &oc::SpaceInformation::allocDirectedControlSampler)
        .def("setDirectedControlSamplerAllocator", &oc::SpaceInformation::setDirectedControlSamplerAllocator, nb::arg("dcsa") )
        .def("clearDirectedSamplerAllocator", &oc::SpaceInformation::clearDirectedSamplerAllocator)

        // StatePropagator
        .def("getStatePropagator", &oc::SpaceInformation::getStatePropagator, nb::rv_policy::reference_internal)

        // setStatePropagator
        .def("setStatePropagator",
            nb::overload_cast<const ompl::control::StatePropagatorFn&>(&ompl::control::SpaceInformation::setStatePropagator),
             nb::arg("sp"))

        // setPropagationStepSize, getPropagationStepSize
        .def("setPropagationStepSize", &oc::SpaceInformation::setPropagationStepSize, nb::arg("stepSize"))
        .def("getPropagationStepSize", &oc::SpaceInformation::getPropagationStepSize)

        // propagate(...) overloads
        .def(
            "propagate",
            [](const oc::SpaceInformation &si, const ob::State *state, const oc::Control *control, int steps,
               ob::State *result) { si.propagate(state, control, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        .def("canPropagateBackward", &oc::SpaceInformation::canPropagateBackward)
        .def(
            "propagateWhileValid",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps,
               ob::State *result) { return si.propagateWhileValid(start, ctrl, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
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
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("alloc") = true)
        .def(
            "propagateWhileValidWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps, bool alloc)
            {
                std::vector<ob::State *> path;
                unsigned int nValid = si.propagateWhileValid(start, ctrl, steps, path, alloc);
                return std::make_pair(nValid, path);
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("alloc") = true)

        // printSettings override
        .def(
            "printSettings", [](const oc::SpaceInformation &si) { si.printSettings(std::cout); })

        // setup override
        .def("setup", &oc::SpaceInformation::setup);
}