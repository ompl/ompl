#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/projections/Identity.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initProjections_Identity(nb::module_ &m)
{
    nb::class_<ml::Projection_Identity, ml::FiberedProjection>(m, "ProjectionIdentity")
        .def(nb::init<ob::StateSpacePtr, ob::StateSpacePtr>(), nb::arg("bundleSpace"), nb::arg("baseSpace"));
}
