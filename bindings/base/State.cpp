#include <nanobind/nanobind.h>
#include "ompl/base/State.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_State(nb::module_& m)
{
    nb::class_<ompl::base::State>(m, "State");
    nb::class_<ompl::base::CompoundState, ompl::base::State>(m, "CompoundState")
        .def(nb::init<>())
            .def("__getitem__",
                [](ompl::base::CompoundState &self, unsigned int i) {
                    return self[i];
                },
                nb::arg("index"), nb::rv_policy::reference_internal)
            .def("__setitem__",
                [](ompl::base::CompoundState *self, unsigned int i, ompl::base::State *s) {
                    self->components[i] = s;
                },
                nb::arg("index"),
                nb::arg("state"));
}
