#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_PRMstar(nb::module_& m)
{
    // TODO [og::PRMstar][IMPLEMENT]
    nb::class_<og::PRMstar, og::PRM>(m, "PRMstar")
        ;
}
