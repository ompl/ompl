#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/ParameterSmoothStep.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_ParameterSmoothStep(nb::module_ &m)
{
    // operator() is not virtual, so the inherited Parameter.__call__ would compute the base (linear) value.
    nb::class_<ompl::ParameterSmoothStep, ompl::Parameter>(m, "ParameterSmoothStep")
        .def(nb::init<>())
        .def(nb::init<double>(), nb::arg("initValue"))
        .def(nb::init<double, double>(), nb::arg("initValue"), nb::arg("targetValue"))
        .def("__call__", &ompl::ParameterSmoothStep::operator());
}
