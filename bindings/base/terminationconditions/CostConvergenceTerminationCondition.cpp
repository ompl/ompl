#include <nanobind/nanobind.h>
#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "../init.hh"

namespace nb = nanobind;

void ompl::binding::base::initTerminationconditions_CostConvergenceTerminationCondition(nb::module_& m)
{
    // TODO [ob::CostConvergenceTerminationCondition][IMPLEMENT]
    nb::class_<ompl::base::CostConvergenceTerminationCondition,
               ompl::base::PlannerTerminationCondition>(m, "CostConvergenceTerminationCondition")
        ;
}
