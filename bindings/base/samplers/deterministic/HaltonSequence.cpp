#include <nanobind/nanobind.h>
#include "ompl/base/samplers/deterministic/HaltonSequence.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersDeterministic_HaltonSequence(nb::module_& m)
{
    // TODO [ob::HaltonSequence][IMPLEMENT]
    nb::class_<ob::HaltonSequence, ob::DeterministicSequence>(m, "HaltonSequence");
}
