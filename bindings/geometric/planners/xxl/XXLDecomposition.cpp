#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/xxl/XXLDecomposition.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXLDecomposition(nb::module_& m)
{
    // TODO [og::XXLDecomposition][IMPLEMENT]
    nb::class_<og::XXLDecomposition>(m, "XXLDecomposition")
        ;
}
