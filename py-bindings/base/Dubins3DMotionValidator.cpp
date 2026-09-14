#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/Dubins3DMotionValidator.h"
#include "ompl/base/spaces/OwenStateSpace.h"
#include "ompl/base/spaces/VanaStateSpace.h"
#include "ompl/base/spaces/VanaOwenStateSpace.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Dubins3DMotionValidator(nb::module_ &m)
{
    nb::class_<ob::Dubins3DMotionValidator<ob::OwenStateSpace>, ob::MotionValidator>(m, "OwenMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));

    nb::class_<ob::Dubins3DMotionValidator<ob::VanaStateSpace>, ob::MotionValidator>(m, "VanaMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));

    nb::class_<ob::Dubins3DMotionValidator<ob::VanaOwenStateSpace>, ob::MotionValidator>(m, "VanaOwenMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));
}
