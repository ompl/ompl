#include <nanobind/nanobind.h>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "../init.hh"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_RealVectorBounds(nb::module_& m)
{
nb::class_<ompl::base::RealVectorBounds>(m, "RealVectorBounds", 
    "Represents bounds for an R^n space with lower and upper limits")
    .def(nb::init<unsigned int>(), "Constructor. dim represents the dimension of the space")
    .def("setLow", nb::overload_cast<double>(&ompl::base::RealVectorBounds::setLow),
         "Set the lower bound in each dimension to a specific value")
    .def("setLow", nb::overload_cast<unsigned int, double>(&ompl::base::RealVectorBounds::setLow),
         "Set the lower bound of a specific dimension to a value")
    .def("setHigh", nb::overload_cast<double>(&ompl::base::RealVectorBounds::setHigh),
         "Set the upper bound in each dimension to a specific value")
    .def("setHigh", nb::overload_cast<unsigned int, double>(&ompl::base::RealVectorBounds::setHigh),
         "Set the upper bound of a specific dimension to a value")
    .def("resize", &ompl::base::RealVectorBounds::resize,
         "Change the number of dimensions for the bounds")
    .def("getVolume", &ompl::base::RealVectorBounds::getVolume,
         "Compute the volume of the space enclosed by the bounds")
    .def("getDifference", &ompl::base::RealVectorBounds::getDifference,
         "Get the difference between high and low bounds for each dimension")
    .def("check", &ompl::base::RealVectorBounds::check,
         "Check if bounds are valid (same length for low/high, high[i] > low[i])")
    .def_rw("low", &ompl::base::RealVectorBounds::low,
         "Lower bounds vector")
    .def_rw("high", &ompl::base::RealVectorBounds::high,
         "Upper bounds vector");
}
