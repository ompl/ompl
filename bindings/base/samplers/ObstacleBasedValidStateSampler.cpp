#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_ObstacleBasedValidStateSampler(nb::module_& m)
{
    // TODO [ob::ObstacleBasedValidStateSampler][TEST]
    nb::class_<ob::ObstacleBasedValidStateSampler, ob::ValidStateSampler>(m, "ObstacleBasedValidStateSampler")
        .def(nb::init<const ob::SpaceInformation*>())
        .def("sample", &ob::ObstacleBasedValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::ObstacleBasedValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"), nb::arg("distance"));
}
