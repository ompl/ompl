#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <ompl/multilevel/datastructures/projections/SE2RN_R2.h>
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initProjections_SE2RN_R2(nb::module_ &m)
{
    nb::class_<om::Projection_SE2RN_R2, om::FiberedProjection>(m, "SE2RN_R2")
        .def(nb::init<ob::StateSpacePtr, ob::StateSpacePtr>(), nb::arg("bundleSpace"), nb::arg("baseSpace"));
}
