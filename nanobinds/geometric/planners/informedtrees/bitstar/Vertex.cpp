#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/bitstar/Vertex.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesBitstar_Vertex(nb::module_& m)
{
    // TODO [og::BITstar::Vertex][IMPLEMENT]
    nb::class_<og::BITstar::Vertex>(m, "Vertex")
        ;
}
