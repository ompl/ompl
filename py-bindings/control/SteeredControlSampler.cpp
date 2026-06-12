#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/SteeredControlSampler.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_SteeredControlSampler(nb::module_ &m)
{
    nb::class_<oc::SteeredControlSampler, oc::DirectedControlSampler>(m, "SteeredControlSampler")
        .def(nb::init<const oc::SpaceInformation *>(), nb::arg("si"))
        .def("sampleTo",
             nb::overload_cast<oc::Control *, const ob::State *, ob::State *>(&oc::SteeredControlSampler::sampleTo),
             nb::arg("control"), nb::arg("source"), nb::arg("dest"))
        .def("sampleTo",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *, ob::State *>(
                 &oc::SteeredControlSampler::sampleTo),
             nb::arg("control"), nb::arg("previous"), nb::arg("source"), nb::arg("dest"));
}
