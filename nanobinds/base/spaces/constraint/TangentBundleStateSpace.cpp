#include <nanobind/nanobind.h>
#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"
#include <nanobind/stl/shared_ptr.h>
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_TangentBundleStateSpace(nb::module_& m)
{
    // TODO [ob::TangentBundleStateSpace][IMPLEMENT]
    nb::class_<ob::TangentBundleStateSpace, ob::AtlasStateSpace>(m, "TangentBundleStateSpace")
        .def(nb::init<const ob::StateSpacePtr&, const ob::ConstraintPtr &>(), nb::arg("baseSpace"), nb::arg("tangentSpace"))
        ;
}
