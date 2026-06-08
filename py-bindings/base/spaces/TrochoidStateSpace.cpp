#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/spaces/TrochoidStateSpace.h"
#include "ompl/base/spaces/DubinsMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_TrochoidStateSpace(nb::module_ &m)
{
    nb::enum_<ob::TrochoidStateSpace::TrochoidPathSegmentType>(m, "TrochoidPathSegmentType")
        .value("TROCHOID_LEFT", ob::TrochoidStateSpace::TROCHOID_LEFT)
        .value("TROCHOID_STRAIGHT", ob::TrochoidStateSpace::TROCHOID_STRAIGHT)
        .value("TROCHOID_RIGHT", ob::TrochoidStateSpace::TROCHOID_RIGHT);

    nb::class_<ob::TrochoidStateSpace::PathType>(m, "TrochoidPathType")
        .def("length", &ob::TrochoidStateSpace::PathType::length)
        .def_rw("reverse_", &ob::TrochoidStateSpace::PathType::reverse_);

    nb::class_<ob::TrochoidStateSpace, ob::SE2StateSpace>(m, "TrochoidStateSpace")
        .def(nb::init<double, double, double, bool>(), nb::arg("turningRadius") = 1.0, nb::arg("windRatio") = 0.0,
             nb::arg("windDirection") = 0.0, nb::arg("isSymmetric") = false)
        .def("distance",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::TrochoidStateSpace::distance, nb::const_))
        .def(
            "getPath",
            nb::overload_cast<const ob::State *, const ob::State *, bool>(&ob::TrochoidStateSpace::getPath, nb::const_),
            nb::arg("state1"), nb::arg("state2"), nb::arg("periodic") = false)
        .def("isMetricSpace", &ob::TrochoidStateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::TrochoidStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::TrochoidStateSpace::hasSymmetricInterpolate)
        .def("validSegmentCount", &ob::TrochoidStateSpace::validSegmentCount)
        .def("allocState", &ob::TrochoidStateSpace::allocState);

    nb::class_<ob::DubinsMotionValidator<ob::TrochoidStateSpace>, ob::MotionValidator>(m, "TrochoidMotionValidator")
        .def(nb::init<ob::SpaceInformation *>());
}
