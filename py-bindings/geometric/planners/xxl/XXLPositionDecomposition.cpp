#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/xxl/XXLPositionDecomposition.h"
#include "PyXXLDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using PyXXLPositionDecomposition = ompl::binding::geometric::PyXXLDecompositionT<og::XXLPositionDecomposition>;

void ompl::binding::geometric::initPlannersXxl_XXLPositionDecomposition(nb::module_ &m)
{
    nb::class_<og::XXLPositionDecomposition, og::XXLDecomposition, PyXXLPositionDecomposition>(
        m, "XXLPositionDecomposition")
        .def(nb::init<const ob::RealVectorBounds &, const std::vector<int> &, bool>(), nb::arg("bounds"),
             nb::arg("slices"), nb::arg("diagonalEdges") = false)
        .def(
            "ridToGridCell",
            [](const og::XXLPositionDecomposition &d, int rid)
            {
                std::vector<int> cell;
                d.ridToGridCell(rid, cell);
                return cell;
            },
            nb::arg("rid"))
        .def("gridCellToRid", &og::XXLPositionDecomposition::gridCellToRid, nb::arg("cell"))
        .def("coordToRegion",
             nb::overload_cast<const std::vector<double> &>(&og::XXLPositionDecomposition::coordToRegion, nb::const_),
             nb::arg("coord"))
        .def("hasDiagonalEdges", &og::XXLPositionDecomposition::hasDiagonalEdges);
}
