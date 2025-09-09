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
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::geometric::init_PathGeometric(nb::module_ &m)
{
    nb::class_<ompl::geometric::PathGeometric, ompl::base::Path>(m, "PathGeometric")
        // Constructors
        .def(nb::init<const ompl::base::SpaceInformationPtr &>(), nb::arg("si"),
             "Construct a PathGeometric given a SpaceInformation pointer")
        .def(nb::init<const ompl::geometric::PathGeometric &>(), "Copy constructor")
        .def(nb::init<const ompl::base::SpaceInformationPtr &, const ompl::base::State *>(), nb::arg("si"),
             nb::arg("state"), "Construct a PathGeometric from a single state")
        .def(nb::init<const ompl::base::SpaceInformationPtr &, const ompl::base::State *, const ompl::base::State *>(),
             nb::arg("si"), nb::arg("state1"), nb::arg("state2"), "Construct a PathGeometric from two states")
        .def(nb::init<const ompl::base::SpaceInformationPtr &, std::vector<const ompl::base::State *> &>(),
             nb::arg("si"), nb::arg("states"), "Construct a PathGeometric from a list of states")

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
        .def("cost", &ompl::geometric::PathGeometric::cost, nb::rv_policy::reference_internal, nb::arg("obj"),
             "Return the cost of the path given an OptimizationObjective")
        .def("length", &ompl::geometric::PathGeometric::length, "Return the length of the path")
        .def("check", &ompl::geometric::PathGeometric::check, "Check if the path is valid")

        // Smoothness and clearance.
        .def("smoothness", &ompl::geometric::PathGeometric::smoothness, "Return the smoothness measure of the path")
        .def("clearance", &ompl::geometric::PathGeometric::clearance, "Return the clearance of the path")

        // Printing.
        .def(
            "print", [](const ompl::geometric::PathGeometric &path) { path.print(std::cout); },
            "Return a string representation of the path")
        .def(
            "printAsMatrix", [](const ompl::geometric::PathGeometric &path) { 
               // path.printAsMatrix(std::cout); 
               std::ostringstream oss;
               path.printAsMatrix(oss);
               return oss.str();},
            "Return a matrix representation of the path")

        // Interpolation (two overloads).
        .def("interpolate", nb::overload_cast<unsigned int>(&ompl::geometric::PathGeometric::interpolate),
             nb::arg("count"), "Interpolate the path by inserting 'count' new states")
        .def("interpolate", nb::overload_cast<>(&ompl::geometric::PathGeometric::interpolate),
             "Interpolate the path using the current settings")

        // Other operations.
        .def("subdivide", &ompl::geometric::PathGeometric::subdivide, "Subdivide the path")
        .def("reverse", &ompl::geometric::PathGeometric::reverse, "Reverse the path")
        .def("checkAndRepair", &ompl::geometric::PathGeometric::checkAndRepair,
             "Check the path and repair it if possible", nb::arg("attempts"))
        .def("overlay", &ompl::geometric::PathGeometric::overlay, "Overlay another path onto this path",
             nb::arg("over"), nb::arg("startIndex") = 0)

        // Append, prepend, and keeping parts.
        .def("append", nb::overload_cast<const ompl::base::State *>(&ompl::geometric::PathGeometric::append),
             nb::arg("state"), "Append a state to the path")
        .def("append",
             nb::overload_cast<const ompl::geometric::PathGeometric &>(&ompl::geometric::PathGeometric::append),
             nb::arg("path"), "Append another path to this path")
        .def("prepend", &ompl::geometric::PathGeometric::prepend, "Prepend a state to the path", nb::arg("state"))
        .def("keepAfter", &ompl::geometric::PathGeometric::keepAfter,
             "Keep the segment of the path after the given state", nb::arg("state"))
        .def("keepBefore", &ompl::geometric::PathGeometric::keepBefore,
             "Keep the segment of the path before the given state", nb::arg("state"))

        // Randomization and validity.
        .def("random", &ompl::geometric::PathGeometric::random, "Randomize the path")
        .def("randomValid", &ompl::geometric::PathGeometric::randomValid,
             "Randomize the path until a valid one is found", nb::arg("attempts"))
        .def("getClosestIndex", &ompl::geometric::PathGeometric::getClosestIndex,
             "Return the index of the state closest to a given state", nb::arg("state"))

        // Getters for states.
        .def(
            "getStates", [](ompl::geometric::PathGeometric &self) -> std::vector<ompl::base::State *>
            { return self.getStates(); }, nb::rv_policy::reference_internal, "Return the list of states in the path")
        .def("getState", nb::overload_cast<unsigned int>(&ompl::geometric::PathGeometric::getState), nb::arg("index"),
             nb::rv_policy::reference_internal, "Return the state at the specified index")
        .def("getStateCount", &ompl::geometric::PathGeometric::getStateCount, "Return the number of states in the path")

        // Clearing the path.
        .def("clear", &ompl::geometric::PathGeometric::clear, "Clear all states from the path")

        // Add a string representation for the PathGeometric class
        .def("__repr__", [](const ompl::geometric::PathGeometric &self) { 
               std::ostringstream oss;
               self.print(oss);
               return oss.str();
         });
}