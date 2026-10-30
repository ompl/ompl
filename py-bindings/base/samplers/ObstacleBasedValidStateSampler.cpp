#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "base/samplers/OwnerRef.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_ObstacleBasedValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::ObstacleBasedValidStateSampler, ob::ValidStateSampler,
               WithOwner<ob::ObstacleBasedValidStateSampler>>(
        m, "ObstacleBasedValidStateSampler", nb::type_slots(ownerSlots<ob::ObstacleBasedValidStateSampler>))
        .def("__init__", initWithOwner<ob::ObstacleBasedValidStateSampler, ob::SpaceInformation>(), nb::arg("si"))
        .def("sample", &ob::ObstacleBasedValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::ObstacleBasedValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"));
}
