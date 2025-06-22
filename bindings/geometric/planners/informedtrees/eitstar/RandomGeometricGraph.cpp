#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/eitstar/RandomGeometricGraph.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesEitstar_RandomGeometricGraph(nb::module_& m)
{
    // TODO [og::eitstar::RandomGeometricGraph][IMPLEMENT]
    nb::class_<og::eitstar::RandomGeometricGraph>(m, "RandomGeometricGraph")
        ;
}
