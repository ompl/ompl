#include <nanobind/nanobind.h>
#include "ompl/control/SimpleDirectedControlSampler.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_SimpleDirectedControlSampler(nb::module_ &m)
{
    nb::class_<oc::SimpleDirectedControlSampler, oc::DirectedControlSampler>(m, "SimpleDirectedControlSampler");
}
