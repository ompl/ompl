#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersKpiece_BKPIECE1(nb::module_& m)
{
    // TODO [og::BKPIECE1][IMPLEMENT]
    // TAG [og::BKPIECE1][Planner]
    nb::class_<og::BKPIECE1, ob::Planner>(m, "BKPIECE1")
        ;
}
