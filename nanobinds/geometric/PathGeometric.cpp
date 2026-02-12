#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/base/Path.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/OptimizationObjective.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::geometric::init_PathGeometric(nb::module_ &m)
{
    nb::class_<ompl::geometric::PathGeometric, ompl::base::Path>(m, "PathGeometric")
        // Constructors
        .def(nb::init<const ompl::base::SpaceInformationPtr &>(), nb::arg("si"))
        .def(nb::init<const ompl::geometric::PathGeometric &>())
        .def(nb::init<const ompl::base::SpaceInformationPtr &, const ompl::base::State *>(), nb::arg("si"),
             nb::arg("state"))
        .def(nb::init<const ompl::base::SpaceInformationPtr &, const ompl::base::State *, const ompl::base::State *>(),
             nb::arg("si"), nb::arg("state1"), nb::arg("state2"))
        .def(nb::init<const ompl::base::SpaceInformationPtr &, std::vector<const ompl::base::State *> &>(),
             nb::arg("si"), nb::arg("states"))

        // Assignment operator
        .def(
            "assign",
            [](ompl::geometric::PathGeometric &self,
               const ompl::geometric::PathGeometric &other) -> ompl::geometric::PathGeometric &
            {
                self = other;
                return self;
            },
            "Assign another PathGeometric to this one")

        // Cost, length, and check.
        .def("cost", &ompl::geometric::PathGeometric::cost, nb::rv_policy::reference_internal, nb::arg("obj"))
        .def("length", &ompl::geometric::PathGeometric::length)
        .def("check", &ompl::geometric::PathGeometric::check)

        // Smoothness and clearance.
        .def("smoothness", &ompl::geometric::PathGeometric::smoothness)
        .def("clearance", &ompl::geometric::PathGeometric::clearance)

        // Printing.
        .def("print", [](const ompl::geometric::PathGeometric &path) { path.print(std::cout); })
        .def("__str__",
             [](const ompl::geometric::PathGeometric &path)
             {
                 std::ostringstream oss;
                 path.printAsMatrix(oss);
                 return oss.str();
             })
        .def("__repr__",
             [](const ompl::geometric::PathGeometric &path)
             {
                 std::ostringstream oss;
                 path.printAsMatrix(oss);
                 return oss.str();
             })
        .def("printAsMatrix", [](const ompl::geometric::PathGeometric &path) { path.printAsMatrix(std::cout); })

        // Interpolation (two overloads).
        .def("interpolate", nb::overload_cast<unsigned int>(&ompl::geometric::PathGeometric::interpolate),
             nb::arg("count"))
        .def("interpolate", nb::overload_cast<>(&ompl::geometric::PathGeometric::interpolate))

        // Other operations.
        .def("subdivide", &ompl::geometric::PathGeometric::subdivide)
        .def("reverse", &ompl::geometric::PathGeometric::reverse)
        .def("checkAndRepair", &ompl::geometric::PathGeometric::checkAndRepair, nb::arg("attempts"))
        .def("overlay", &ompl::geometric::PathGeometric::overlay, nb::arg("over"), nb::arg("startIndex") = 0)

        // Append, prepend, and keeping parts.
        .def("append", nb::overload_cast<const ompl::base::State *>(&ompl::geometric::PathGeometric::append),
             nb::arg("state"))
        .def("append",
             nb::overload_cast<const ompl::geometric::PathGeometric &>(&ompl::geometric::PathGeometric::append),
             nb::arg("path"))
        .def("prepend", &ompl::geometric::PathGeometric::prepend, nb::arg("state"))
        .def("keepAfter", &ompl::geometric::PathGeometric::keepAfter, nb::arg("state"))
        .def("keepBefore", &ompl::geometric::PathGeometric::keepBefore, nb::arg("state"))

        // Randomization and validity.
        .def("random", &ompl::geometric::PathGeometric::random)
        .def("randomValid", &ompl::geometric::PathGeometric::randomValid, nb::arg("attempts"))
        .def("getClosestIndex", &ompl::geometric::PathGeometric::getClosestIndex, nb::arg("state"))

        // Getters for states.
        .def(
            "getStates", [](ompl::geometric::PathGeometric &self) -> std::vector<ompl::base::State *>
            { return self.getStates(); }, nb::rv_policy::reference_internal)
        .def("getState", nb::overload_cast<unsigned int>(&ompl::geometric::PathGeometric::getState), nb::arg("index"),
             nb::rv_policy::reference_internal)
        .def("getStateCount", &ompl::geometric::PathGeometric::getStateCount)

        // Clearing the path.
        .def("clear", &ompl::geometric::PathGeometric::clear)

        // Add a string representation for the PathGeometric class
        .def("__repr__",
             [](const ompl::geometric::PathGeometric &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             });
}