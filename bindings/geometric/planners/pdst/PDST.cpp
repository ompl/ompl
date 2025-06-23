#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/pdst/PDST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPdst_PDST(nb::module_& m)
{
    // TODO [og::PDST][IMPLEMENT]
    // TAG [og::PDST][Planner]
    nb::class_<og::PDST, ob::Planner>(m, "PDST")
        ;
}
