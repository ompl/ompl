#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/ABITstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_ABITstar(nb::module_& m)
{
    // TODO [og::ABITstar][IMPLEMENT]
    // TAG [og::ABITstar][Planner]
    nb::class_<og::ABITstar, og::BITstar>(m, "ABITstar")
        ;
}
