#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/est/ProjEST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersEst_ProjEST(nb::module_& m)
{
    // TODO [og::ProjEST][IMPLEMENT]
    // TAG [og::ProjEST][Planner]
    nb::class_<og::ProjEST, ob::Planner>(m, "ProjEST")
        ;
}
