#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/DubinsMotionValidator.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/TrochoidStateSpace.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_DubinsMotionValidator(nb::module_ &m)
{
    nb::class_<ob::DubinsMotionValidator<ob::DubinsStateSpace>, ob::MotionValidator>(m, "DubinsMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));

    nb::class_<ob::DubinsMotionValidator<ob::ReedsSheppStateSpace>, ob::MotionValidator>(m, "ReedsSheppMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));

    nb::class_<ob::DubinsMotionValidator<ob::TrochoidStateSpace>, ob::MotionValidator>(m, "TrochoidMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));
}
