#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/samplers/MaximizeClearanceValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"

#include "../init.hh"

namespace nb  = nanobind;
namespace ob  = ompl::base;

void ompl::binding::base::initSamplers_MaximizeClearanceValidStateSampler(nb::module_ &m) {
    // TODO [ob::MaximizeClearanceValidStateSampler][TEST]
    nb::class_<ob::MaximizeClearanceValidStateSampler,
               ob::ValidStateSampler>(m, "MaximizeClearanceValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(),
             nb::arg("si"),
             "Construct a MaximizeClearanceValidStateSampler for the given SpaceInformation.")
        .def("sample",
             &ob::MaximizeClearanceValidStateSampler::sample,
             nb::arg("state"),
             "Try to sample a valid state that maximizes clearance.  Returns True on success.")
        .def("sampleNear",
             &ob::MaximizeClearanceValidStateSampler::sampleNear,
             nb::arg("state"),
             nb::arg("near"),
             nb::arg("distance"),
             "Try to sample near a given state (within `distance`), maximizing clearance.")
        .def("setNrImproveAttempts",
             &ob::MaximizeClearanceValidStateSampler::setNrImproveAttempts,
             nb::arg("attempts"),
             "Set how many local-improvement attempts each sample will do.")
        .def("getNrImproveAttempts",
             &ob::MaximizeClearanceValidStateSampler::getNrImproveAttempts,
             "Get the number of local-improvement attempts configured.");
}
