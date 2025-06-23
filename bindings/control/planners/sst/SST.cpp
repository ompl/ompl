#include <nanobind/nanobind.h>
#include "ompl/control/planners/sst/SST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSst_SST(nb::module_& m)
{
    // TODO [oc::SST][IMPLEMENT]
    // TAG [oc::SST][Planner]
    nb::class_<oc::SST, ob::Planner>(m, "SST")
        ;
}
