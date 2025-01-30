#include <nanobind/nanobind.h>
#include "ompl/util/ProlateHyperspheroid.h"
namespace nb = nanobind;

void init_prolate_hyperspheroid(nb::module_& m) {
    // Expose ProlateHyperspheroid class
    nb::class_<ompl::ProlateHyperspheroid>(m, "ProlateHyperspheroid")
        .def(nb::init<unsigned int, const double*, const double*>())
        .def("set_transverse_diameter", &ompl::ProlateHyperspheroid::setTransverseDiameter)
        .def("transform", &ompl::ProlateHyperspheroid::transform)
        .def("is_in_phs", &ompl::ProlateHyperspheroid::isInPhs)
        .def("is_on_phs", &ompl::ProlateHyperspheroid::isOnPhs)
        .def("get_phs_dimension", &ompl::ProlateHyperspheroid::getPhsDimension)
        .def("get_phs_measure", nb::overload_cast<>(&ompl::ProlateHyperspheroid::getPhsMeasure, nb::const_))
        .def("get_min_transverse_diameter", &ompl::ProlateHyperspheroid::getMinTransverseDiameter)
        .def("get_path_length", &ompl::ProlateHyperspheroid::getPathLength)
        .def("get_dimension", &ompl::ProlateHyperspheroid::getDimension);
}