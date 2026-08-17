#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "ompl/control/planners/ltl/World.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::initPlannersLtl_World(nb::module_ &m)
{
    nb::class_<oc::World>(m, "World")
        .def(nb::init<unsigned int>(), nb::arg("numProps"))
        .def(
            "__getitem__", [](const oc::World &w, unsigned int i) { return w[i]; }, nb::arg("index"))
        .def(
            "__setitem__", [](oc::World &w, unsigned int i, bool value) { w[i] = value; }, nb::arg("index"),
            nb::arg("value"))
        .def("numProps", &oc::World::numProps)
        .def("satisfies", &oc::World::satisfies, nb::arg("w"))
        .def("formula", &oc::World::formula)
        .def("clear", &oc::World::clear);
}
