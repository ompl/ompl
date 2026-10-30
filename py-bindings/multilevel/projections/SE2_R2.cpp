#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/projections/SE2_R2.h"
#include "detail.hpp"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initProjections_SE2_R2(nb::module_ &m)
{
    detail::bindFiberedProjection<ml::Projection_SE2_R2>(m, "ProjectionSE2R2");
}
