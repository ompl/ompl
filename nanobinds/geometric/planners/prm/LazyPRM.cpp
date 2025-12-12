#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_LazyPRM(nb::module_& m)
{
    // TODO [og::LazyPRM][IMPLEMENT]
    // TAG [og::LazyPRM][Planner]
    nb::class_<og::LazyPRM, ob::Planner>(m, "LazyPRM")
        ;
}
