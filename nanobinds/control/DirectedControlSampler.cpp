#include <nanobind/nanobind.h>
#include "ompl/control/DirectedControlSampler.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_DirectedControlSampler(nb::module_& m)
{
    nb::class_<oc::DirectedControlSampler>(m, "DirectedControlSampler")
        ;
}
