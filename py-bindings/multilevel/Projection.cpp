#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/Projection.h"
#include "ompl/multilevel/datastructures/projections/FiberedProjection.h"
#include "init.h"

namespace nb = nanobind;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::init_Projection(nb::module_ &m)
{
    nb::class_<ml::Projection>(m, "Projection");
    nb::class_<ml::FiberedProjection, ml::Projection>(m, "FiberedProjection");
}
