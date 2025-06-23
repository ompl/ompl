#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/EIRMstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_EIRMstar(nb::module_& m)
{
    // TODO [og::EIRMstar][IMPLEMENT]
    // TAG [og::EIRMstar][Planner]
    nb::class_<og::EIRMstar, og::EITstar>(m, "EIRMstar")
        ;
}
