#include <nanobind/nanobind.h>

#include "ompl/geometric/planners/xxl/XXLPositionDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXLPositionDecomposition(nb::module_ &m)
{
    nb::class_<og::XXLPositionDecomposition, og::XXLDecomposition>(m, "XXLPositionDecomposition");
}
