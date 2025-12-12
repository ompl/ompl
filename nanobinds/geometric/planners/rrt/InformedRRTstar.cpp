#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_InformedRRTstar(nb::module_& m)
{
    // TODO [og::InformedRRTstar][IMPLEMENT]
    // TAG [og::InformedRRTstar][Planner]
    nb::class_<og::InformedRRTstar, og::RRTstar>(m, "InformedRRTstar")
        ;
}
