#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/ParameterExponentialDecay.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_ParameterExponentialDecay(nb::module_ &m)
{
    // operator() is not virtual, so the inherited Parameter.__call__ would compute the base (linear) value.
    nb::class_<ompl::ParameterExponentialDecay, ompl::Parameter>(m, "ParameterExponentialDecay")
        .def(nb::init<>())
        .def(nb::init<double>(), nb::arg("lambda_"))
        .def(nb::init<double, double>(), nb::arg("lambda_"), nb::arg("valueInit"))
        .def(nb::init<double, double, double>(), nb::arg("lambda_"), nb::arg("valueInit"), nb::arg("valueTarget"))
        .def("__call__", &ompl::ParameterExponentialDecay::operator())
        .def("setLambda", &ompl::ParameterExponentialDecay::setLambda, nb::arg("lambda_"));
}
