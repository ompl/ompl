#include <nanobind/nanobind.h>
#include "ompl/base/samplers/deterministic/PrecomputedSequence.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersDeterministic_PrecomputedSequence(nb::module_& m)
{
    nb::class_<ob::PrecomputedSequence, ob::DeterministicSequence>(m, "PrecomputedSequence");
}
