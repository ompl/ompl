#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rrt/TSRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_TSRRT(nb::module_& m)
{
    // TODO [og::TaskSpaceConfig][IMPLEMENT]
    nb::class_<og::TaskSpaceConfig>(m, "TaskSpaceConfig")
        ;

    // TODO [og::TSRRT][IMPLEMENT]
    // TAG [og::TSRRT][Planner]
    nb::class_<og::TSRRT, ob::Planner>(m, "TSRRT")
        ;
}
