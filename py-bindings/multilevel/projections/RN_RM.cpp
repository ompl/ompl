#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/multilevel/datastructures/projections/RN_RM.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initProjections_RN_RM(nb::module_ &m)
{
    nb::class_<ml::Projection_RN_RM, ml::FiberedProjection>(m, "ProjectionRNRM")
        .def(nb::init<ob::StateSpacePtr, ob::StateSpacePtr>(), nb::arg("bundleSpace"), nb::arg("baseSpace"));
}
