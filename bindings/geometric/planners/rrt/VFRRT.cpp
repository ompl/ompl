#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_VFRRT(nb::module_& m)
{
    // TODO [og::VFRRT][IMPLEMENT]
    // TAG [og::VFRRT][Planner]
    nb::class_<og::VFRRT, og::RRT>(m, "VFRRT")
        ;
}
