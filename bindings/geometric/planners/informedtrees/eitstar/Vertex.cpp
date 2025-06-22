#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/eitstar/Vertex.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesEitstar_Vertex(nb::module_& m)
{
    nb::class_<og::eitstar::Vertex>(m, "Vertex")
        ;
}
