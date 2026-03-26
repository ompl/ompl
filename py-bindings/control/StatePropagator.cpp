#include <nanobind/nanobind.h>
#include "ompl/control/StatePropagator.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::init_StatePropagator(nb::module_ &m)
{
    nb::class_<oc::StatePropagator>(m, "StatePropagator");
}
