#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/pRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_pRRT(nb::module_& m)
{
    // TODO [og::pRRT][IMPLEMENT]
    // TAG [og::pRRT][Planner]
    nb::class_<og::pRRT, ob::Planner>(m, "pRRT")
        ;
}
