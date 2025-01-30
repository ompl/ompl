#include <nanobind/nanobind.h>
#include "ompl/base/StateSpace.h"

namespace nb = nanobind;

void initStateSpace(nb::module_& m) {
    nb::class_<ompl::base::StateSpace>(m, "StateSpace");
    nb::class_<ompl::base::CompoundStateSpace, ompl::base::StateSpace>(m, "CompoundStateSpace")
        .def(nb::init<>());
}