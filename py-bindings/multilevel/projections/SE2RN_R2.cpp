#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <ompl/multilevel/datastructures/projections/SE2RN_R2.h>
#include "detail.hpp"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initProjections_SE2RN_R2(nb::module_ &m)
{
    detail::bindFiberedProjection<om::Projection_SE2RN_R2>(m, "SE2RN_R2");
}
