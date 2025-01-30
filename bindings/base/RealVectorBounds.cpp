#include <nanobind/nanobind.h>
#include "ompl/base/spaces/RealVectorBounds.h"

namespace nb = nanobind;

void initRealVectorBounds(nb::module_& m) {
    nb::class_<ompl::base::RealVectorBounds>(m, "RealVectorBounds")
        .def(nb::init<unsigned int>())
        .def("setLow", nb::overload_cast<double>(&ompl::base::RealVectorBounds::setLow))
        .def("setLow", nb::overload_cast<unsigned int, double>(&ompl::base::RealVectorBounds::setLow))
        .def("setHigh", nb::overload_cast<double>(&ompl::base::RealVectorBounds::setHigh))
        .def("setHigh", nb::overload_cast<unsigned int, double>(&ompl::base::RealVectorBounds::setHigh))
        .def("resize", &ompl::base::RealVectorBounds::resize)
        .def("getVolume", &ompl::base::RealVectorBounds::getVolume)
        .def("getDifference", &ompl::base::RealVectorBounds::getDifference)
        .def("check", &ompl::base::RealVectorBounds::check)
        .def_rw("low", &ompl::base::RealVectorBounds::low)
        .def_rw("high", &ompl::base::RealVectorBounds::high);
}