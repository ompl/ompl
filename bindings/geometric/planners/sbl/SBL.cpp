#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/sbl/SBL.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersSbl_SBL(nb::module_& m)
{
    // TODO [og::SBL][IMPLEMENT]
    // TAG [og::SBL][Planner]
    nb::class_<og::SBL, ob::Planner>(m, "SBL")
        ;
}
