#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/ParameterExponentialDecay.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_ParameterExponentialDecay(nb::module_ &m)
{
    nb::class_<ompl::ParameterExponentialDecay, ompl::Parameter>(m, "ParameterExponentialDecay").def(nb::init<>());
}
