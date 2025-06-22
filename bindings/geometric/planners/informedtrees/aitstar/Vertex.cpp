#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/aitstar/Vertex.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesAitstar_Vertex(nb::module_& m)
{
    nb::class_<og::aitstar::Vertex>(m, "Vertex")
        ;
}
