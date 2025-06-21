#include <nanobind/nanobind.h>
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersInformed_PathLengthDirectInfSampler(nb::module_& m)
{
    // TODO [ob::PathLengthDirectInfSampler][IMPLEMENT]
    nb::class_<ob::PathLengthDirectInfSampler, ob::InformedSampler>(m, "PathLengthDirectInfSampler");
}
