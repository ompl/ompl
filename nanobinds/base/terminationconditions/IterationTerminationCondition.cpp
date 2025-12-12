#include <nanobind/nanobind.h>
#include "ompl/base/terminationconditions/IterationTerminationCondition.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initTerminationconditions_IterationTerminationCondition(nb::module_& m)
{
    // TODO [ob::IterationTerminationCondition][IMPLEMENT]
    nb::class_<ob::IterationTerminationCondition>(m, "IterationTerminationCondition");
}
