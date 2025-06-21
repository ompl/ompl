#include <nanobind/nanobind.h>
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersInformed_RejectionInfSampler(nb::module_& m)
{
    // TODO [ob::RejectionInfSampler][IMPLEMENT]
    nb::class_<ob::RejectionInfSampler, ob::InformedSampler>(m, "RejectionInfSampler");
}
