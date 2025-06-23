#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/prm/LazyPRMstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_LazyPRMstar(nb::module_& m)
{
    // TODO [og::LazyPRMstar][IMPLEMENT]
    // TAG [og::LazyPRMstar][Planner]
    nb::class_<og::LazyPRMstar, og::LazyPRM>(m, "LazyPRMstar")
        ;
}
