#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARStwo(nb::module_& m)
{
    // TODO [og::SPARStwo][IMPLEMENT]
    // TAG [og::SPARStwo][Planner]
    nb::class_<og::SPARStwo, ob::Planner>(m, "SPARStwo")
        ;
}
