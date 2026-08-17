#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/xxl/XXLDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXLDecomposition(nb::module_ &m)
{
    nb::class_<og::XXLDecomposition>(m, "XXLDecomposition")
        .def("getNumRegions", &og::XXLDecomposition::getNumRegions)
        .def("getDimension", &og::XXLDecomposition::getDimension)
        .def("numLayers", &og::XXLDecomposition::numLayers)
        .def("locateRegion", nb::overload_cast<const ob::State *>(&og::XXLDecomposition::locateRegion, nb::const_),
             nb::arg("s"))
        .def("locateRegion",
             nb::overload_cast<const std::vector<double> &>(&og::XXLDecomposition::locateRegion, nb::const_),
             nb::arg("coord"))
        .def("getNeighbors", &og::XXLDecomposition::getNeighbors, nb::arg("rid"), nb::arg("neighbors"))
        .def("getNeighborhood", &og::XXLDecomposition::getNeighborhood, nb::arg("rid"), nb::arg("neighborhood"))
        .def("distanceHeuristic", &og::XXLDecomposition::distanceHeuristic, nb::arg("r1"), nb::arg("r2"))
        .def("canSteer", &og::XXLDecomposition::canSteer);
}
