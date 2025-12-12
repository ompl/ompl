#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/cforest/CForestStateSampler.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::geometric::initPlannersCforest_CForestStateSampler(nb::module_& m)
{
    // TODO [og::CForestStateSampler][IMPLEMENT]
    nb::class_<ob::CForestStateSampler, ob::StateSampler>(m, "CForestStateSampler")
        ;

    
}
