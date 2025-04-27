#include <nanobind/nanobind.h>
#include "ompl/base/samplers/DeterministicStateSampler.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_DeterministicStateSampler(nb::module_& m)
{
    // TODO [ob::DeterministicStateSampler][IMPLEMENT]
    // TODO [ob::DeterministicStateSampler][TRAMPOLINE]
    nb::class_<ob::DeterministicStateSampler, ob::StateSampler>(m, "DeterministicStateSampler");
}
