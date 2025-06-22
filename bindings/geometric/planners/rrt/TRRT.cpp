#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/TRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_TRRT(nb::module_& m)
{
    // TODO [og::TRRT][IMPLEMENT]
    nb::class_<og::TRRT, ob::Planner>(m, "TRRT")
        ;
}
