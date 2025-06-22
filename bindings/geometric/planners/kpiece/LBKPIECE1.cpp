#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersKpiece_LBKPIECE1(nb::module_& m)
{
    // TODO [og::LBKPIECE1][IMPLEMENT]
    nb::class_<og::LBKPIECE1, ob::Planner>(m, "LBKPIECE1")
        ;
}
