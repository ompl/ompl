#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/kpiece/Discretization.h"
#include "ompl/base/State.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Define a generic Motion type for Python bindings
struct PyMotion
{
    ob::State *state{nullptr};
    PyMotion *parent{nullptr};
};

// Instantiate Discretization with PyMotion
using PyDiscretization = og::Discretization<PyMotion>;

void ompl::binding::geometric::initPlannersKpiece_Discretization(nb::module_ &m)
{
    // Bind the CellData struct
    nb::class_<PyDiscretization::CellData>(m, "DiscretizationCellData")
        .def(nb::init<>())
        .def_ro("coverage", &PyDiscretization::CellData::coverage)
        .def_ro("selections", &PyDiscretization::CellData::selections)
        .def_ro("score", &PyDiscretization::CellData::score)
        .def_ro("iteration", &PyDiscretization::CellData::iteration)
        .def_ro("importance", &PyDiscretization::CellData::importance);

    // Bind the Discretization class with a default no-op free function
    nb::class_<PyDiscretization>(m, "Discretization")
        .def("__init__", [](PyDiscretization *self) { new (self) PyDiscretization([](PyMotion *) {}); })

        // Border fraction
        .def("setBorderFraction", &PyDiscretization::setBorderFraction, nb::arg("bp"))
        .def("getBorderFraction", &PyDiscretization::getBorderFraction)

        // Dimension
        .def("setDimension", &PyDiscretization::setDimension, nb::arg("dim"))

        // Clear
        .def("clear", &PyDiscretization::clear)

        // Iteration
        .def("countIteration", &PyDiscretization::countIteration)

        // Counts
        .def("getMotionCount", &PyDiscretization::getMotionCount)
        .def("getCellCount", &PyDiscretization::getCellCount)

        // Free memory
        .def("freeMemory", &PyDiscretization::freeMemory);
}

