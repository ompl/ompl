#include <nanobind/nanobind.h>
#include "ompl/control/planners/pdst/PDST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersPdst_PDST(nb::module_& m)
{
    // TODO [oc::PDST][IMPLEMENT]
    // TAG [oc::PDST][Planner]
    nb::class_<oc::PDST, ob::Planner>(m, "PDST")
        ;
}
