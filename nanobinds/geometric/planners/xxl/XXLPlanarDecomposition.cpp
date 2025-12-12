#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/xxl/XXLPlanarDecomposition.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXLPlanarDecomposition(nb::module_& m)
{
    // TODO [og::XXLPlanarDecomposition][IMPLEMENT]
    nb::class_<og::XXLPlanarDecomposition, og::XXLDecomposition>(m, "XXLPlanarDecomposition")
        ;
}
