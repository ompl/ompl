#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/control/PathControl.h"
#include "ompl/base/OptimizationObjective.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

void ompl::binding::control::init_PathControl(nb::module_ &m)
{
    nb::class_<oc::PathControl, ob::Path>(m, "PathControl")
        // Constructors
        .def(nb::init<const ob::SpaceInformationPtr &>(),
             nb::arg("si"),
             "Construct a PathControl given a SpaceInformation.")
        .def(nb::init<const oc::PathControl &>(),
             "Copy constructor")

        // cost(OptimizationObjectivePtr)
        // If you have not bound base::Cost, either skip or wrap the return differently.
        .def("cost",
             &oc::PathControl::cost,
             nb::arg("opt"),
             "Compute the cost of this path under the provided optimization objective.")

        // length(), check() override from base::Path
        .def("length", &oc::PathControl::length,
             "Return the geometric length of the path (sum of durations doesn't necessarily match).")
        .def("check", &oc::PathControl::check,
             "Check if the path is valid under the underlying space information constraints.")

        // print() returns void, so let's wrap it returning a string
        .def("print",
             [](const oc::PathControl &pc) {
                 pc.print(std::cout);
             },
             "Return a string describing this path.")

        // printAsMatrix() also returns void; wrap similarly
        .def("printAsMatrix",
             [](const oc::PathControl &pc) {
                pc.printAsMatrix(std::cout);
             },
             "Return a matrix-form string describing the path's states & controls.")

        // asGeometric() returns a geometric::PathGeometric by value
        // We can return that by value or copy. The default is fine.
        .def("asGeometric", &oc::PathControl::asGeometric,
             "Convert this PathControl into a geometric PathGeometric object.")

        // append(const State*), append(const State*, const Control*, double)
        .def("append",
             nb::overload_cast<const ob::State *>(&oc::PathControl::append),
             nb::arg("state"),
             "Append a state at the end of the path.")
        .def("append",
             nb::overload_cast<const ob::State *, const oc::Control *, double>(&oc::PathControl::append),
             nb::arg("state"), nb::arg("control"), nb::arg("duration"),
             "Append a state, control, and duration at the end of the path.")

        // Additional methods
        .def("interpolate", &oc::PathControl::interpolate,
             "Interpolate the path's states & controls as needed.")
        .def("random", &oc::PathControl::random,
             "Randomize the path.")
        .def("randomValid", &oc::PathControl::randomValid,
             nb::arg("attempts"),
             "Attempt to randomize the path such that it remains valid, up to the given number of tries.")

        // Expose the internal vectors via a direct reference. The vector<...> is automatically converted to a Python list.
        .def("getStates",
             &oc::PathControl::getStates,
             nb::rv_policy::reference_internal,
             "Return a list (std::vector) of state pointers in the path.")
        .def("getControls",
             &oc::PathControl::getControls,
             nb::rv_policy::reference_internal,
             "Return a list (std::vector) of control pointers in the path.")
        .def("getControlDurations",
             &oc::PathControl::getControlDurations,
             nb::rv_policy::reference_internal,
             "Return a list (std::vector) of control durations in the path.")

        // Single-element getters
        .def("getState",
             nb::overload_cast<unsigned int>(&oc::PathControl::getState),
             nb::arg("index"),
             nb::rv_policy::reference_internal,
             "Return the i-th state in the path (non-const).")

        .def("getControl",
             nb::overload_cast<unsigned int>(&oc::PathControl::getControl),
             nb::arg("index"),
             nb::rv_policy::reference_internal,
             "Return the i-th control (non-const).")

        .def("getControlDuration",
             &oc::PathControl::getControlDuration,
             nb::arg("index"),
             "Return the duration of the i-th control segment.")

        // Count queries
        .def("getStateCount", &oc::PathControl::getStateCount,
             "Return the number of states in the path.")
        .def("getControlCount", &oc::PathControl::getControlCount,
             "Return the number of controls in the path.");
}