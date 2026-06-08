#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/DubinsMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_DubinsStateSpace(nb::module_ &m)
{
    nb::enum_<ob::DubinsStateSpace::DubinsPathSegmentType>(m, "DubinsPathSegmentType")
        .value("DUBINS_LEFT", ob::DubinsStateSpace::DUBINS_LEFT)
        .value("DUBINS_STRAIGHT", ob::DubinsStateSpace::DUBINS_STRAIGHT)
        .value("DUBINS_RIGHT", ob::DubinsStateSpace::DUBINS_RIGHT);

    nb::class_<ob::DubinsStateSpace::PathType>(m, "DubinsPathType")
        .def("length", &ob::DubinsStateSpace::PathType::length)
        .def_rw("reverse_", &ob::DubinsStateSpace::PathType::reverse_);

    nb::class_<ob::DubinsStateSpace, ob::SE2StateSpace>(m, "DubinsStateSpace")
        .def(nb::init<double, bool>(), nb::arg("turningRadius") = 1.0, nb::arg("isSymmetric") = false)
        .def("distance",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::DubinsStateSpace::distance, nb::const_))
        .def("getPath",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::DubinsStateSpace::getPath, nb::const_))
        .def("isMetricSpace", &ob::DubinsStateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::DubinsStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::DubinsStateSpace::hasSymmetricInterpolate)
        .def("validSegmentCount", &ob::DubinsStateSpace::validSegmentCount)
        .def("allocState", &ob::DubinsStateSpace::allocState);

    nb::class_<ob::DubinsMotionValidator<ob::DubinsStateSpace>, ob::MotionValidator>(m, "DubinsMotionValidator")
        .def(nb::init<ob::SpaceInformation *>());
}
