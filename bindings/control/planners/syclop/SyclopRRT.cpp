#include <nanobind/nanobind.h>
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_SyclopRRT(nb::module_& m)
{
    // TODO [oc::SyclopRRT][IMPLEMENT]
    nb::class_<oc::SyclopRRT, oc::Syclop>(m, "SyclopRRT")
        ;
}
