#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/prm/PRM.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_PRM(nb::module_& m)
{
    // TODO [og::PRM][IMPLEMENT]
    // TAG [og::PRM][Planner]
    nb::class_<og::PRM, ob::Planner>(m, "PRM")
        ;
}
