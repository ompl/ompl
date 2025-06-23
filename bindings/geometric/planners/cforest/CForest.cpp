#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/cforest/CForest.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersCforest_CForest(nb::module_& m)
{
    // TODO [og::CForest][IMPLEMENT]
    // TAG [og::CForest][Planner]
    nb::class_<og::CForest, ob::Planner>(m, "CForest")
        ;
}
