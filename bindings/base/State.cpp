#include <nanobind/nanobind.h>
#include "ompl/base/State.h"

namespace nb = nanobind;

void initState(nb::module_& m) {
    // Base State class
    nb::class_<ompl::base::State>(m, "State")
        .def("__init__", [](ompl::base::State*) {
            throw nb::type_error("State class cannot be instantiated directly");
        });

    // CompoundState class
    nb::class_<ompl::base::CompoundState, ompl::base::State>(m, "CompoundState")
        .def("__getitem__", [](const ompl::base::CompoundState& self, unsigned int i) {
            if (self.components == nullptr || self.components[i] == nullptr)
                throw nb::index_error("Invalid component access");
            return self.components[i];
        })
        .def("__setitem__", [](ompl::base::CompoundState& self, unsigned int i, 
                              ompl::base::State* state) {
            if (self.components == nullptr)
                throw nb::index_error("Components array not initialized");
            self.components[i] = state;
        });
}