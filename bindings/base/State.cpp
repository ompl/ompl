#include <nanobind/nanobind.h>
#include "ompl/base/State.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_State(nb::module_& m)
{
    nb::class_<ompl::base::State>(m, "State");
    nb::class_<ompl::base::CompoundState, ompl::base::State>(m, "CompoundState");
}
