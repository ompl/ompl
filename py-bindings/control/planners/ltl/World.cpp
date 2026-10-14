#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>

#include "ompl/control/planners/ltl/World.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::initPlannersLtl_World(nb::module_ &m)
{
    nb::class_<oc::World> world(m, "World");
    world.def(nb::init<unsigned int>(), nb::arg("numProps"))
        .def(
            "__getitem__",
            [](const oc::World &w, unsigned int i)
            {
                auto it = w.props().find(i);
                if (it == w.props().end())
                    throw nb::key_error(("proposition " + std::to_string(i) + " is not set in this world").c_str());
                return it->second;
            },
            nb::arg("index"))
        .def(
            "__setitem__", [](oc::World &w, unsigned int i, bool value) { w[i] = value; }, nb::arg("index"),
            nb::arg("value"))
        .def("numProps", &oc::World::numProps)
        .def("satisfies", &oc::World::satisfies, nb::arg("w"))
        .def("formula", &oc::World::formula)
        .def("clear", &oc::World::clear)
        .def(nb::self == nb::self);
    // Mutable and compared by value, so not hashable.
    world.attr("__hash__") = nb::none();
}
