#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/BridgeTestValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ValidStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_BridgeTestValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::BridgeTestValidStateSampler, ob::ValidStateSampler>(m, "BridgeTestValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(), nb::arg("si"))
        .def("sample", &ob::BridgeTestValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::BridgeTestValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("getStdDev", &ob::BridgeTestValidStateSampler::getStdDev)
        .def("setStdDev", &ob::BridgeTestValidStateSampler::setStdDev, nb::arg("stdDev"));
}
