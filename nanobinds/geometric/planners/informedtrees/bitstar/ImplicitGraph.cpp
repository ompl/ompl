#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/bitstar/ImplicitGraph.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesBitstar_ImplicitGraph(nb::module_& m)
{
    // TODO [og::BITstar::ImplicitGraph][IMPLEMENT]
    nb::class_<og::BITstar::ImplicitGraph>(m, "ImplicitGraph")
        ;
}
