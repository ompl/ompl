#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_DiscreteMotionValidator(nb::module_ &m)
{
    nb::class_<ob::DiscreteMotionValidator, ob::MotionValidator>(m, "DiscreteMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));
}
