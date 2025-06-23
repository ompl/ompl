#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/AITstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_AITstar(nb::module_& m)
{
    // TODO [og::AITstar][IMPLEMENT]
    // TAG [og::AITstar][Planner]
    nb::class_<og::AITstar, ob::Planner>(m, "AITStar")
        ;
}
