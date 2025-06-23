#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_LazyRRT(nb::module_& m)
{
    // TODO [og::LazyRRT][IMPLEMENT]
    // TAG [og::LazyRRT][Planner]
    nb::class_<og::LazyRRT, ob::Planner>(m, "LazyRRT")
        ;
}
