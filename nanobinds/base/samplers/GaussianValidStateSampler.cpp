#include <nanobind/nanobind.h>
#include "ompl/base/samplers/GaussianValidStateSampler.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_GaussianValidStateSampler(nb::module_& m)
{
    // TODO [ob::GaussianValidStateSampler][IMPLEMENT]
    nb::class_<ob::GaussianValidStateSampler, ob::ValidStateSampler>(m, "GaussianValidStateSampler");
}
