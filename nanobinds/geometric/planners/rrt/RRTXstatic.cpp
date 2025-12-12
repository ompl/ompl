#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/RRTXstatic.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRTXstatic(nb::module_& m)
{
    // TODO [og::RRTXstatic][IMPLEMENT]
    // TAG [og::RRTXstatic][Planner]
    nb::class_<og::RRTXstatic, ob::Planner>(m, "RRTXstatic")
        ;
}
