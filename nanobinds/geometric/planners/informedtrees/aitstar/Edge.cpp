#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/aitstar/Edge.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesAitstar_Edge(nb::module_& m)
{
    // TODO [og::aitstar::Edge][IMPLEMENT]
    nb::class_<og::aitstar::Edge>(m, "Edge")
        ;
}
