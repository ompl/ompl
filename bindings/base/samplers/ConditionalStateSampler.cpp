#include <nanobind/nanobind.h>
#include "ompl/base/samplers/ConditionalStateSampler.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_ConditionalStateSampler(nb::module_& m)
{
    // TODO [ob::ConditionalStateSampler][IMPLEMENT]
    nb::class_<ob::ConditionalStateSampler, ob::ValidStateSampler>(m, "ConditionalStateSampler");
}
