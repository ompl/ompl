#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/MaximizeClearanceValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ValidStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_MaximizeClearanceValidStateSampler(nb::module_ &m)
{
    nb::class_<ob::MaximizeClearanceValidStateSampler, ob::ValidStateSampler>(m, "MaximizeClearanceValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(), nb::arg("si"))
        .def("sample", &ob::MaximizeClearanceValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::MaximizeClearanceValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("setNrImproveAttempts", &ob::MaximizeClearanceValidStateSampler::setNrImproveAttempts,
             nb::arg("attempts"))
        .def("getNrImproveAttempts", &ob::MaximizeClearanceValidStateSampler::getNrImproveAttempts);
}
