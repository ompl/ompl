#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRTstar(nb::module_& m)
{
    // TODO [og::RRTstar][IMPLEMENT]
    // TAG [og::RRTstar][Planner]
    nb::class_<og::RRTstar, ob::Planner>(m, "RRTstar")
        ;
}
