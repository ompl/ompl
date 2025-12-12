#include <nanobind/nanobind.h>
#include "ompl/base/samplers/BridgeTestValidStateSampler.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_BridgeTestValidStateSampler(nb::module_& m)
{
    // TODO [ob::BridgeTestValidStateSampler][IMPLEMENT]
    nb::class_<ob::BridgeTestValidStateSampler, ob::ValidStateSampler>(m, "BridgeTestValidStateSampler");
}
