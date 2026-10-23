#include <nanobind/nanobind.h>
#include "ompl/control/DirectedControlSampler.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_DirectedControlSampler(nb::module_ &m)
{
    nb::class_<oc::DirectedControlSampler>(m, "DirectedControlSampler")
        .def("sampleTo",
             nb::overload_cast<oc::Control *, const ob::State *, ob::State *>(&oc::DirectedControlSampler::sampleTo),
             nb::arg("control"), nb::arg("source"), nb::arg("dest"))
        .def("sampleTo",
             nb::overload_cast<oc::Control *, const oc::Control *, const ob::State *, ob::State *>(
                 &oc::DirectedControlSampler::sampleTo),
             nb::arg("control"), nb::arg("previous"), nb::arg("source"), nb::arg("dest"));
}
