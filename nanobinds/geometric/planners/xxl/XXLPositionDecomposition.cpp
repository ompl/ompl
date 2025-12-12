#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/xxl/XXLPositionDecomposition.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXLPositionDecomposition(nb::module_& m)
{
    // TODO [og::XXLPositionDecomposition][IMPLEMENT]
    nb::class_<og::XXLPositionDecomposition, og::XXLDecomposition>(m, "XXLPositionDecomposition")
        ;
}
