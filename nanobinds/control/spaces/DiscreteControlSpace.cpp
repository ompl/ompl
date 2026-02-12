#include <nanobind/nanobind.h>
#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::control::initSpaces_DiscreteControlSpace(nb::module_ &m)
{
    nb::class_<ompl::control::DiscreteControlSampler, ompl::control::ControlSampler>(m, "DiscreteControlSampler");

    nb::class_<ompl::control::DiscreteControlSpace, ompl::control::ControlSpace>(m, "DiscreteControlSpace");
}
