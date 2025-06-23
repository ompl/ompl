#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/sst/SST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersSst_SST(nb::module_& m)
{
    // TODO [og::SST][IMPLEMENT]
    // TAG [og::SST][Planner]
    nb::class_<og::SST, ob::Planner>(m, "SST")
        ;
}
