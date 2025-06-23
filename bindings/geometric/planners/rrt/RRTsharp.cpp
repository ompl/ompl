#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/RRTsharp.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRTsharp(nb::module_& m)
{
    // TODO [og::RRTsharp][IMPLEMENT]
    // TAG [og::RRTsharp][Planner]
    nb::class_<og::RRTsharp, og::RRTXstatic>(m, "RRTsharp")
        ;
}
