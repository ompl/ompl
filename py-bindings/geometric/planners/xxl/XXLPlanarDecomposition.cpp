#include <nanobind/nanobind.h>

#include "ompl/geometric/planners/xxl/XXLPlanarDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXLPlanarDecomposition(nb::module_ &m)
{
    nb::class_<og::XXLPlanarDecomposition, og::XXLDecomposition>(m, "XXLPlanarDecomposition");
}
