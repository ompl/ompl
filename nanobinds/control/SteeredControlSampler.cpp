#include <nanobind/nanobind.h>
#include "ompl/control/SteeredControlSampler.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_SteeredControlSampler(nb::module_& m)
{
    // TODO [oc::SteeredControlSampler][IMPLEMENT]
    nb::class_<oc::SteeredControlSampler, oc::DirectedControlSampler>(m, "SteeredControlSampler")
        ;
}
