#include <nanobind/nanobind.h>
#include "ompl/control/planners/syclop/Decomposition.h"
#include "../../init.hh"

namespace nb = nanobind;

void ompl::binding::control::initPlannersSyclop_Decomposition(nb::module_& m)
{
    // TODO [oc::Decomposition][IMPLEMENT]
    nb::class_<ompl::control::Decomposition>(m, "Decomposition")
        ;
}
