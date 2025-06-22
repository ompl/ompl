#include <nanobind/nanobind.h>
#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "../init.hh"

namespace nb = nanobind;

void ompl::binding::control::initSpaces_DiscreteControlSpace(nb::module_& m)
{
    // TODO [oc::DiscreteControlSampler][IMPLEMENT]
    nb::class_<ompl::control::DiscreteControlSampler, ompl::control::ControlSampler>(m, "DiscreteControlSampler")
        ;

    // TODO [oc::DiscreteControlSpace][IMPLEMENT]
    nb::class_<ompl::control::DiscreteControlSpace, ompl::control::ControlSpace>(m, "DiscreteControlSpace")
        ;
}
