#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_LBTRRT(nb::module_& m)
{
    // TODO [og::LBTRRT][IMPLEMENT]
    // TAG [og::LBTRRT][Planner]
    nb::class_<og::LBTRRT, ob::Planner>(m, "LBTRRT")
        ;
}
