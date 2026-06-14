#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/util/ProlateHyperspheroid.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_ProlateHyperspheroid(nb::module_ &m)
{
    nb::class_<ompl::ProlateHyperspheroid>(m, "ProlateHyperspheroid")
        .def(nb::new_([](unsigned int n, const std::vector<double> &focus1, const std::vector<double> &focus2)
                      { return new ompl::ProlateHyperspheroid(n, focus1.data(), focus2.data()); }),
             nb::arg("n"), nb::arg("focus1"), nb::arg("focus2"))
        .def("setTransverseDiameter", &ompl::ProlateHyperspheroid::setTransverseDiameter, nb::arg("transverseDiameter"))
        .def(
            "transform",
            [](const ompl::ProlateHyperspheroid &phs, const std::vector<double> &sphere)
            {
                std::vector<double> result(sphere.size());
                phs.transform(sphere.data(), result.data());
                return result;
            },
            nb::arg("sphere"))
        .def(
            "isInPhs", [](const ompl::ProlateHyperspheroid &phs, const std::vector<double> &point)
            { return phs.isInPhs(point.data()); }, nb::arg("point"))
        .def(
            "isOnPhs", [](const ompl::ProlateHyperspheroid &phs, const std::vector<double> &point)
            { return phs.isOnPhs(point.data()); }, nb::arg("point"))
        .def("getPhsDimension", &ompl::ProlateHyperspheroid::getPhsDimension)
        .def("getPhsMeasure", nb::overload_cast<>(&ompl::ProlateHyperspheroid::getPhsMeasure, nb::const_))
        .def("getPhsMeasure", nb::overload_cast<double>(&ompl::ProlateHyperspheroid::getPhsMeasure, nb::const_),
             nb::arg("tranDiam"))
        .def("getMinTransverseDiameter", &ompl::ProlateHyperspheroid::getMinTransverseDiameter)
        .def(
            "getPathLength", [](const ompl::ProlateHyperspheroid &phs, const std::vector<double> &point)
            { return phs.getPathLength(point.data()); }, nb::arg("point"))
        .def("getDimension", &ompl::ProlateHyperspheroid::getDimension);
}
