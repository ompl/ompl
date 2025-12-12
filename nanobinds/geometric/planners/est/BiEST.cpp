#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/est/BiEST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersEst_BiEST(nb::module_& m)
{
    // TODO [og::BiEST][IMPLEMENT]
    // TAG [og::BiEST][Planner]
    nb::class_<og::BiEST, ob::Planner>(m, "BiEST")
        ;
}
