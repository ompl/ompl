#include <nanobind/nanobind.h>
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersInformed_OrderedInfSampler(nb::module_& m)
{
    // TODO [ob::OrderedInfSampler][IMPLEMENT]
    nb::class_<ob::OrderedInfSampler, ob::InformedSampler>(m, "OrderedInfSampler");
}
