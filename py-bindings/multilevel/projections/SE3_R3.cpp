#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/projections/SE3_R3.h"
#include "detail.hpp"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initProjections_SE3_R3(nb::module_ &m)
{
    detail::bindFiberedProjection<ml::Projection_SE3_R3>(m, "ProjectionSE3R3");
}
