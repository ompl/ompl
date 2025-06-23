#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/rlrt/RLRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRlrt_RLRT(nb::module_& m)
{
    // TODO [og::RLRT][IMPLEMENT]
    // TAG [og::RLRT][Planner]
    nb::class_<og::RLRT, ob::Planner>(m, "RLRT")
        ;
}
