#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/fmt/FMT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersFmt_FMT(nb::module_& m)
{
    // TODO [og::FMT][IMPLEMENT]
    // TAG [og::FMT][Planner]
    nb::class_<og::FMT, ob::Planner>(m, "FMT")
        ;
}
