#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/projections/Identity.h"
#include "detail.hpp"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initProjections_Identity(nb::module_ &m)
{
    detail::bindFiberedProjection<ml::Projection_Identity>(m, "ProjectionIdentity");
}
