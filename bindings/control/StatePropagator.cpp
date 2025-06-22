#include <nanobind/nanobind.h>
#include "ompl/control/StatePropagator.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::control::init_StatePropagator(nb::module_& m)
{
    // TODO [ompl::control::StatePropagator][IMPLEMENT]
    nb::class_<ompl::control::StatePropagator>(m, "StatePropagator");
}
