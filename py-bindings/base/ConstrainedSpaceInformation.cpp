#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>

#include "ompl/base/ConstrainedSpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_ConstrainedSpaceInformation(nb::module_ &m)
{
    nb::class_<ob::ConstrainedValidStateSampler, ob::ValidStateSampler>(m, "ConstrainedValidStateSampler");

    nb::class_<ob::ConstrainedSpaceInformation, ob::SpaceInformation>(m, "ConstrainedSpaceInformation")
        .def(nb::init<ob::StateSpacePtr>(), nb::arg("space"))
        .def("getMotionStates", &ob::ConstrainedSpaceInformation::getMotionStates);

    nb::class_<ob::TangentBundleSpaceInformation, ob::ConstrainedSpaceInformation>(m, "TangentBundleSpaceInformation")
        .def(nb::init<ob::StateSpacePtr>(), nb::arg("space"));
}
