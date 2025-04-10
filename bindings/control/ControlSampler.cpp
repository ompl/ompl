#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/control/ControlSampler.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;


void ompl::binding::control::init_ControlSampler(nb::module_& m)
{
    nb::class_<ompl::control::ControlSampler>(m, "ControlSampler");

    nb::class_<oc::CompoundControlSampler, oc::ControlSampler>(m, "CompoundControlSampler")
        .def(nb::init<const oc::ControlSpace*>(),
             nb::arg("space"),
             "Construct a CompoundControlSampler for the given control space")
        
        // Overloaded sample() methods:
        .def("sample", 
             nb::overload_cast<oc::Control *>(&oc::CompoundControlSampler::sample, nb::const_),
             nb::arg("control"),
             "Sample a control uniformly")
        
        .def("sample", 
             nb::overload_cast<oc::Control *, const ob::State *>(&oc::CompoundControlSampler::sample, nb::const_),
             nb::arg("control"), nb::arg("state"),
             "Sample a control uniformly near the given state")
        
        // Overloaded sampleNext() methods:
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *>(&oc::CompoundControlSampler::sampleNext, nb::const_),
             nb::arg("control"), nb::arg("previous"),
             "Sample the next control based on the previous control")
        
        .def("sampleNext",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *>(&oc::CompoundControlSampler::sampleNext, nb::const_),
             nb::arg("control"), nb::arg("previous"), nb::arg("state"),
             "Sample the next control based on the previous control and a given state")
        ;
}
