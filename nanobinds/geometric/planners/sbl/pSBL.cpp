#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/sbl/pSBL.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersSbl_pSBL(nb::module_& m)
{
    // TODO [og::pSBL][IMPLEMENT]
    // TAG [og::pSBL][Planner]
    nb::class_<og::pSBL, ob::Planner>(m, "pSBL")
        ;
}
