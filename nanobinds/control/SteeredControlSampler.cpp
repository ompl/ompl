#include <nanobind/nanobind.h>
#include "ompl/control/SteeredControlSampler.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_SteeredControlSampler(nb::module_ &m)
{
    nb::class_<oc::SteeredControlSampler, oc::DirectedControlSampler>(m, "SteeredControlSampler");
}
