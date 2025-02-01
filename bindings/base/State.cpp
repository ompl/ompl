#include <nanobind/nanobind.h>
#include "ompl/base/State.h"
#include <ompl/base/spaces/SE2StateSpace.h>

namespace nb = nanobind;

void initState(nb::module_& m) {
    // Base State class
    nb::class_<ompl::base::State>(m, "State");
        // .def("castTo", [](const ompl::base::State &self) {
        //     return nb::cpp_function([&self](const nb::type_object& type) -> nb::object {
        //         if (nb::isinstance<ompl::base::SE2StateSpace::StateType>(nb::cast(&self)))
        //             // return nb::cast(static_cast<const ompl::base::SE2StateSpace::StateType*>(&self));
        //             return nb::cast(static_cast<const ompl::base::SE2StateSpace::StateType*>(&self));
        //         throw nb::type_error("Unsupported type for 'as' method");
        //     });
        // });

    // CompoundState class
    nb::class_<ompl::base::CompoundState, ompl::base::State>(m, "CompoundState");
}