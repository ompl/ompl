#include <nanobind/nanobind.h>
#include "ompl/control/planners/pdst/PDST.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersPdst_PDST(nb::module_ &m)
{
    nb::class_<oc::PDST, ob::Planner>(m, "PDST");
}
