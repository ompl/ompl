#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/BiTRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_BiTRRT(nb::module_& m)
{
    // TODO [og::BiTRRT][IMPLEMENT]
    // TAG [og::BiTRRT][Planner]
    nb::class_<og::BiTRRT, ob::Planner>(m, "BiTRRT")
        ;
}
