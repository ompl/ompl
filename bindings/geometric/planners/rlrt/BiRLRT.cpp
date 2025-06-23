#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rlrt/BiRLRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRlrt_BiRLRT(nb::module_& m)
{
    // TODO [og::BiRLRT][IMPLEMENT]
    // TAG [og::BiRLRT][Planner]
    nb::class_<og::BiRLRT, ob::Planner>(m, "BiRLRT")
        ;
}
