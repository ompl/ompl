#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/projections/SE2_R2.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initProjections_SE2_R2(nb::module_ &m)
{
    nb::class_<ml::Projection_SE2_R2, ml::FiberedProjection>(m, "ProjectionSE2R2")
        .def(nb::init<ob::StateSpacePtr, ob::StateSpacePtr>(), nb::arg("bundleSpace"), nb::arg("baseSpace"));
}
