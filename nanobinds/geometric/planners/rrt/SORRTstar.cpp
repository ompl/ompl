#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/SORRTstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_SORRTstar(nb::module_& m)
{
    // TODO [og::SORRTstar][IMPLEMENT]
    // TAG [og::SORRTstar][Planner]
    nb::class_<og::SORRTstar, og::InformedRRTstar>(m, "SORRTstar")
        ;
}
