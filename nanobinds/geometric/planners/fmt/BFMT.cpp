#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/fmt/BFMT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersFmt_BFMT(nb::module_& m)
{
    // TODO [og::BFMT][IMPLEMENT]
    // TAG [og::BFMT][Planner]
    nb::class_<og::BFMT, ob::Planner>(m, "BFMT")
        ;
}
