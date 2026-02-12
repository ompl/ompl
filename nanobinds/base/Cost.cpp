#include <nanobind/nanobind.h>
#include "ompl/base/Cost.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::base::init_Cost(nb::module_ &m)
{
    nb::class_<ompl::base::Cost>(m, "Cost")
        .def(nb::init<double>())
        .def("value", &ompl::base::Cost::value)
        .def("__repr__", [](const ompl::base::Cost &c) { return std::to_string(c.value()); });
}
