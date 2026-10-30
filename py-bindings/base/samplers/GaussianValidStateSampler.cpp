#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/GaussianValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ValidStateSampler.h"
#include "base/samplers/OwnerRef.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_GaussianValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::GaussianValidStateSampler, ob::ValidStateSampler, WithOwner<ob::GaussianValidStateSampler>>(
        m, "GaussianValidStateSampler", nb::type_slots(ownerSlots<ob::GaussianValidStateSampler>))
        .def("__init__", initWithOwner<ob::GaussianValidStateSampler, ob::SpaceInformation>(), nb::arg("si"))
        .def("sample", &ob::GaussianValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::GaussianValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("getStdDev", &ob::GaussianValidStateSampler::getStdDev)
        .def("setStdDev", &ob::GaussianValidStateSampler::setStdDev, nb::arg("stdDev"));
}
