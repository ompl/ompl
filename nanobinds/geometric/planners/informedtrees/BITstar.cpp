#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_BITstar(nb::module_& m)
{
    // TODO [og::BITstar][IMPLEMENT]
    // TAG [og::BITstar][Planner]
    nb::class_<og::BITstar, ob::Planner>(m, "BITstar")
        ;
}
