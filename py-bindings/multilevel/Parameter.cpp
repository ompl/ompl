#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/Parameter.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_Parameter(nb::module_ &m)
{
    nb::class_<ompl::Parameter>(m, "Parameter")
        .def(nb::init<>())
        .def(nb::init<double>(), nb::arg("valueInit"))
        .def(nb::init<double, double>(), nb::arg("valueInit"), nb::arg("valueTarget"))
        .def("__call__", &ompl::Parameter::operator())
        .def("setValueInit", &ompl::Parameter::setValueInit, nb::arg("valueInit"))
        .def("setValueTarget", &ompl::Parameter::setValueTarget, nb::arg("valueTarget"))
        .def("setCounterInit", &ompl::Parameter::setCounterInit, nb::arg("counterInit"))
        .def("setCounterTarget", &ompl::Parameter::setCounterTarget, nb::arg("counterTarget"))
        .def("getValueInit", &ompl::Parameter::getValueInit)
        .def("getValueTarget", &ompl::Parameter::getValueTarget)
        .def("getCounterInit", &ompl::Parameter::getCounterInit)
        .def("getCounterTarget", &ompl::Parameter::getCounterTarget)
        .def("getCounter", &ompl::Parameter::getCounter)
        .def("incrementCounter", &ompl::Parameter::incrementCounter)
        .def("reset", &ompl::Parameter::reset);
}
