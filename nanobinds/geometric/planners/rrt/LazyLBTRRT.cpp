#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/LazyLBTRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_LazyLBTRRT(nb::module_& m)
{
    // TODO [og::LazyLBTRRT][IMPLEMENT]
    // TAG [og::LazyLBTRRT][Planner]
    nb::class_<og::LazyLBTRRT, ob::Planner>(m, "LazyLBTRRT")
        ;
}
