#include <nanobind/nanobind.h>
#include "ompl/control/planners/syclop/Syclop.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_Syclop(nb::module_ &m)
{
    nb::class_<oc::Syclop, ob::Planner>(m, "Syclop");
}
