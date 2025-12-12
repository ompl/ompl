#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/est/EST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersEst_EST(nb::module_& m)
{
    // TODO [og::EST][IMPLEMENT]
    // TAG [og::EST][Planner]
    nb::class_<og::EST, ob::Planner>(m, "EST")
        ;
}
