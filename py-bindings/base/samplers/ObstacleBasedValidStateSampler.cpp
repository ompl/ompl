#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_ObstacleBasedValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::ObstacleBasedValidStateSampler, ob::ValidStateSampler>(m, "ObstacleBasedValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(), nb::arg("si"))
        .def("sample", &ob::ObstacleBasedValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::ObstacleBasedValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"));
}
