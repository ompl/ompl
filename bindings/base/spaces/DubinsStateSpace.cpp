#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_DubinsStateSpace(nb::module_ &m)
{
    // TODO [ob::DubinsStateSpace::DubinsPathSegmentType][TEST]
    nb::enum_<ob::DubinsStateSpace::DubinsPathSegmentType>(m, "DubinsPathSegmentType")
        .value("DUBINS_LEFT", ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT)
        .value("DUBINS_STRAIGHT", ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_STRAIGHT)
        .value("DUBINS_RIGHT", ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT);

    // TODO [ob::DubinsStateSpace::DubinsPath][TEST]
    nb::class_<ob::DubinsStateSpace::DubinsPath>(m, "DubinsPath")
        .def(nb::init<>())  // No-arg default constructor
        .def("length", &ob::DubinsStateSpace::DubinsPath::length);

    // TODO [ob::DubinsStateSpace][TEST]
    nb::class_<ob::DubinsStateSpace, ob::SE2StateSpace>(m, "DubinsStateSpace")
        .def(nb::init<double, bool>(), nb::arg("turningRadius") = 1.0, nb::arg("isSymmetric") = false)
        .def("isMetricSpace", &ob::DubinsStateSpace::isMetricSpace)
        .def("distance",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::DubinsStateSpace::distance, nb::const_))
        .def("hasSymmetricDistance", &ob::DubinsStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::DubinsStateSpace::hasSymmetricInterpolate)
        .def("validSegmentCount", &ob::DubinsStateSpace::validSegmentCount)
        .def("sanityChecks", &ob::DubinsStateSpace::sanityChecks)
        .def("dubins",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::DubinsStateSpace::dubins, nb::const_));

    // TODO [ob::DubinsMotionValidator][TEST]
    nb::class_<ob::DubinsMotionValidator, ob::MotionValidator>(m, "DubinsMotionValidator")
        .def(nb::init<ob::SpaceInformation *>())
        .def(nb::init<const ob::SpaceInformationPtr &>())
        .def("checkMotion", nb::overload_cast<const ob::State *, const ob::State *>(
                                &ob::DubinsMotionValidator::checkMotion, nb::const_))
        .def("checkMotion", nb::overload_cast<const ob::State *, const ob::State *, std::pair<ob::State *, double> &>(
                                &ob::DubinsMotionValidator::checkMotion, nb::const_));
}
