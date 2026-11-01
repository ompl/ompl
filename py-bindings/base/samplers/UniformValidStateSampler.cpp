#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "base/samplers/OwnerRef.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_UniformValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::UniformValidStateSampler, ob::ValidStateSampler, WithOwner<ob::UniformValidStateSampler>>(
        m, "UniformValidStateSampler", nb::type_slots(ownerSlots<ob::UniformValidStateSampler>))
        .def("__init__", initWithOwner<ob::UniformValidStateSampler, ob::SpaceInformation>(), nb::arg("si"))
        .def("sample", &ob::UniformValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::UniformValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"));
}
