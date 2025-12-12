#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/ValidStateSampler.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_ValidStateSampler(nb::module_& m)
{
    // TODO [ob::ValidStateSampler][IMPLEMENT]
    nb::class_<ob::ValidStateSampler>(m, "ValidStateSampler");
}
