#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/MinimumClearanceValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ValidStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_MinimumClearanceValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::MinimumClearanceValidStateSampler, ob::ValidStateSampler>(m, "MinimumClearanceValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(), nb::arg("si"))
        .def("sample", &ob::MinimumClearanceValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::MinimumClearanceValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("setMinimumObstacleClearance", &ob::MinimumClearanceValidStateSampler::setMinimumObstacleClearance,
             nb::arg("clearance"))
        .def("getMinimumObstacleClearance", &ob::MinimumClearanceValidStateSampler::getMinimumObstacleClearance);
}
