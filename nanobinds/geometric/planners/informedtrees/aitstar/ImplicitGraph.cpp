#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/aitstar/ImplicitGraph.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesAitstar_ImplicitGraph(nb::module_& m)
{
    // TODO [og::aitstar::ImplicitGraph][IMPLEMENT]
    nb::class_<og::aitstar::ImplicitGraph>(m, "ImplicitGraph")
        ;
}
