#include <nanobind/nanobind.h>
#include "ompl/base/samplers/deterministic/DeterministicSequence.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersDeterministic_DeterministicSequence(nb::module_& m)
{
    // TODO [ob::DeterministicSequence][IMPLEMENT]
    // TODO [ob::DeterministicSequence][TRAMPOLINE]
    nb::class_<ob::DeterministicSequence>(m, "DeterministicSequence");
}
