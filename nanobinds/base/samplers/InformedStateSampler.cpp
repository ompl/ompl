#include <nanobind/nanobind.h>
#include "ompl/base/samplers/InformedStateSampler.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_InformedStateSampler(nb::module_& m)
{
    // TODO [ob::InformedSampler][IMPLEMENT]
    nb::class_<ob::InformedSampler>(m, "InformedSampler");
    // TODO [ob::InformedStateSampler][IMPLEMENT]
    nb::class_<ob::InformedStateSampler, ob::StateSampler>(m, "InformedStateSampler");
}
