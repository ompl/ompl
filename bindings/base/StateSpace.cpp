#include <nanobind/nanobind.h>
#include "ompl/base/StateSpace.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_StateSpace(nb::module_& m)
{
    nb::class_<ompl::base::StateSpace>(m, "StateSpace");
    nb::class_<ompl::base::CompoundStateSpace, ompl::base::StateSpace>(m, "CompoundStateSpace")
        .def(nb::init<>());
}
