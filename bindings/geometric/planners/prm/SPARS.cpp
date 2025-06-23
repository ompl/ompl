#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/prm/SPARS.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARS(nb::module_& m)
{
    // TODO [og::SPARS][IMPLEMENT]
    // TAG [og::SPARS][Planner]
    nb::class_<og::SPARS, ob::Planner>(m, "SPARS")
        ;
}
