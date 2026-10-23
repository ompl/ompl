#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/flat/FlatMotionValidator.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesFlat_FlatMotionValidator(nb::module_ &m)
{
    nb::class_<ob::FlatMotionValidator, ob::MotionValidator>(m, "FlatMotionValidator")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("checkMotion",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::FlatMotionValidator::checkMotion, nb::const_),
             nb::arg("s1"), nb::arg("s2"));
}
