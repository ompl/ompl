#include <nanobind/nanobind.h>
#include "ompl/base/PrecomputedStateSampler.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PrecomputedStateSampler(nb::module_& m)
{
    // TODO [ob::PrecomputedStateSampler][IMPLEMENT]
    nb::class_<ob::PrecomputedStateSampler, ob::StateSampler>(m, "PrecomputedStateSampler")
        ;
}
