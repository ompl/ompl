#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/experience/LightningRetrieveRepair.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersExperience_LightningRetrieveRepair(nb::module_& m)
{
    // TODO [og::LightningRetrieveRepair][IMPLEMENT]
    // TAG [og::LightningRetrieveRepair][Planner]
    nb::class_<og::LightningRetrieveRepair, ob::Planner>(m, "LightningRetrieveRepair")
        ;
}
