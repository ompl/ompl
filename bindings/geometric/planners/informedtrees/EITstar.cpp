#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/EITstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_EITstar(nb::module_& m)
{
    // TODO [og::EITstar][IMPLEMENT]
    // TAG [og::EITstar][Planner]
    nb::class_<og::EITstar, ob::Planner>(m, "EITstar")
        ;
}
