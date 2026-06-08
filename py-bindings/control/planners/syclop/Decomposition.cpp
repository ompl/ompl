#include <nanobind/nanobind.h>
#include "ompl/control/planners/syclop/Decomposition.h"
#include "../../init.h"

namespace nb = nanobind;

void ompl::binding::control::initPlannersSyclop_Decomposition(nb::module_ &m)
{
    nb::class_<ompl::control::Decomposition>(m, "Decomposition");
}
