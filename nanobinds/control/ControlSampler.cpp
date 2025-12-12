#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/control/ControlSampler.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"

#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;


void ompl::binding::control::init_ControlSampler(nb::module_& m)
{
    // TODO [oc::ControlSampler][TRAMPOLINE]
    nb::class_<oc::ControlSampler>(m, "ControlSampler");

    // TODO [oc::UniformControlSampler][TRAMPOLINE]
    nb::class_<oc::CompoundControlSampler, oc::ControlSampler>(m, "CompoundControlSampler")
        .def(nb::init<const oc::ControlSpace*>(),
             nb::arg("space"),
             "Construct a CompoundControlSampler for the given control space")
        // Overloaded sample() methods:
        .def("sample", 
             nb::overload_cast<oc::Control *>(&oc::CompoundControlSampler::sample),
             nb::arg("control"),
             "Sample a control uniformly")
        .def("sample", 
             nb::overload_cast<oc::Control *, const ob::State *>(&oc::CompoundControlSampler::sample),
             nb::arg("control"), nb::arg("state"),
             "Sample a control uniformly near the given state")
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *>(&oc::CompoundControlSampler::sampleNext),
             nb::arg("control"), nb::arg("previous"),
             "Sample the next control based on the previous control")
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *>(&oc::CompoundControlSampler::sampleNext),
             nb::arg("control"), nb::arg("previous"), nb::arg("state"),
             "Sample the next control based on the previous control and a given state");
}
