#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersKpiece_KPIECE1(nb::module_& m)
{
    // TODO [og::KPIECE1][IMPLEMENT]
    nb::class_<og::KPIECE1, ob::Planner>(m, "KPIECE1")
        ;
}
