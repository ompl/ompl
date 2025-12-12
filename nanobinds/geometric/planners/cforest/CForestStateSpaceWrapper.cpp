#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/cforest/CForestStateSpaceWrapper.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::geometric::initPlannersCforest_CForestStateSpaceWrapper(nb::module_& m)
{
    // TODO [og::CForestStateSpaceWrapper][IMPLEMENT]
    nb::class_<ob::CForestStateSpaceWrapper, ob::StateSpace>(m, "CForestStateSpaceWrapper")
        ;
}
