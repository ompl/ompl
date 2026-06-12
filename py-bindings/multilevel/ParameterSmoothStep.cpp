#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/ParameterSmoothStep.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_ParameterSmoothStep(nb::module_ &m)
{
    nb::class_<ompl::ParameterSmoothStep, ompl::Parameter>(m, "ParameterSmoothStep").def(nb::init<>());
}
