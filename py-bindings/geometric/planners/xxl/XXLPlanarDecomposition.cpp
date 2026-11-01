#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/xxl/XXLPlanarDecomposition.h"
#include "PyXXLDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using PyXXLPlanarDecomposition = ompl::binding::geometric::PyXXLDecompositionT<og::XXLPlanarDecomposition>;

void ompl::binding::geometric::initPlannersXxl_XXLPlanarDecomposition(nb::module_ &m)
{
    nb::class_<og::XXLPlanarDecomposition, og::XXLDecomposition, PyXXLPlanarDecomposition>(m, "XXLPlanarDecomposition")
        .def(nb::init<const ob::RealVectorBounds &, const std::vector<int> &, int, bool>(), nb::arg("xyBounds"),
             nb::arg("xySlices"), nb::arg("thetaSlices"), nb::arg("diagonalEdges") = false)
        .def(nb::init<const ob::RealVectorBounds &, const std::vector<int> &, int, double, double, bool>(),
             nb::arg("xyBounds"), nb::arg("xySlices"), nb::arg("thetaSlices"), nb::arg("thetaLowerBound"),
             nb::arg("thetaUpperBound"), nb::arg("diagonalEdges") = false)
        .def(
            "ridToGridCell",
            [](const og::XXLPlanarDecomposition &d, int rid)
            {
                std::vector<int> cell;
                d.ridToGridCell(rid, cell);
                return cell;
            },
            nb::arg("rid"))
        .def("gridCellToRid", &og::XXLPlanarDecomposition::gridCellToRid, nb::arg("cell"))
        .def("coordToRegion",
             nb::overload_cast<const std::vector<double> &>(&og::XXLPlanarDecomposition::coordToRegion, nb::const_),
             nb::arg("coord"))
        .def("hasDiagonalEdges", &og::XXLPlanarDecomposition::hasDiagonalEdges);
}
