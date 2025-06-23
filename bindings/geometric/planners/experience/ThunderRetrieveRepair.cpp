#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/experience/ThunderRetrieveRepair.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersExperience_ThunderRetrieveRepair(nb::module_& m)
{
    // TODO [og::ThunderRetrieveRepair][IMPLEMENT]
    // TAG [og::ThunderRetrieveRepair][Planner]
    nb::class_<og::ThunderRetrieveRepair, ob::Planner>(m, "ThunderRetrieveRepair")
        ;
}
