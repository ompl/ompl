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
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(nb::init<const oc::PathControl &>())

        // cost(OptimizationObjectivePtr)
        // If you have not bound base::Cost, either skip or wrap the return differently.
        .def("cost", &oc::PathControl::cost, nb::arg("opt"))

        // length(), check() override from base::Path
        .def("length", &oc::PathControl::length)
        .def("check", &oc::PathControl::check)

        // print() returns void, so let's wrap it returning a string
        .def("print", [](const oc::PathControl &pc) { pc.print(std::cout); })

        // printAsMatrix() also returns void; wrap similarly
        .def("printAsMatrix",
             [](const oc::PathControl &pc)
             {
               //   pc.printAsMatrix(std::cout);
                 std::ostringstream oss;
                 pc.printAsMatrix(oss);
                 return oss.str();
             })

        // asGeometric() returns a geometric::PathGeometric by value
        // We can return that by value or copy. The default is fine.
        .def("asGeometric", &oc::PathControl::asGeometric)

        // append(const State*), append(const State*, const Control*, double)
        .def("append", nb::overload_cast<const ob::State *>(&oc::PathControl::append), nb::arg("state"))
        .def("append", nb::overload_cast<const ob::State *, const oc::Control *, double>(&oc::PathControl::append),
             nb::arg("state"), nb::arg("control"), nb::arg("duration"))

        // Additional methods
        .def("interpolate", &oc::PathControl::interpolate)
        .def("random", &oc::PathControl::random)
        .def("randomValid", &oc::PathControl::randomValid, nb::arg("attempts"))

        // Expose the internal vectors via a direct reference. The vector<...> is automatically converted to a Python
        // list.
        .def("getStates", &oc::PathControl::getStates, nb::rv_policy::reference_internal)
        .def("getControls", &oc::PathControl::getControls, nb::rv_policy::reference_internal)
        .def("getControlDurations", &oc::PathControl::getControlDurations, nb::rv_policy::reference_internal)

        // Single-element getters
        .def("getState", nb::overload_cast<unsigned int>(&oc::PathControl::getState), nb::arg("index"),
             nb::rv_policy::reference_internal)

        .def("getControl", nb::overload_cast<unsigned int>(&oc::PathControl::getControl), nb::arg("index"),
             nb::rv_policy::reference_internal)

        .def("getControlDuration", &oc::PathControl::getControlDuration, nb::arg("index"))

        // Count queries
        .def("getStateCount", &oc::PathControl::getStateCount)
        .def("getControlCount", &oc::PathControl::getControlCount);
}