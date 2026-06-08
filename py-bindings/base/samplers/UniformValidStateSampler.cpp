#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_UniformValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::UniformValidStateSampler, ob::ValidStateSampler>(m, "UniformValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>())
        .def("sample", &ob::UniformValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::UniformValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"));
}
