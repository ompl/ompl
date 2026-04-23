#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/optional.h>
#include "ompl/base/spaces/OwenStateSpace.h"
#include "ompl/base/spaces/Dubins3DMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_OwenStateSpace(nb::module_ &m)
{
    nb::enum_<ob::OwenStateSpace::PathCategory>(m, "OwenPathCategory")
        .value("LOW_ALTITUDE", ob::OwenStateSpace::PathCategory::LOW_ALTITUDE)
        .value("MEDIUM_ALTITUDE", ob::OwenStateSpace::PathCategory::MEDIUM_ALTITUDE)
        .value("HIGH_ALTITUDE", ob::OwenStateSpace::PathCategory::HIGH_ALTITUDE)
        .value("UNKNOWN", ob::OwenStateSpace::PathCategory::UNKNOWN);

    nb::class_<ob::OwenStateSpace::PathType>(m, "OwenPathType")
        .def("length", &ob::OwenStateSpace::PathType::length)
        .def("category", &ob::OwenStateSpace::PathType::category)
        .def_rw("turnRadius_", &ob::OwenStateSpace::PathType::turnRadius_)
        .def_rw("deltaZ_", &ob::OwenStateSpace::PathType::deltaZ_)
        .def_rw("phi_", &ob::OwenStateSpace::PathType::phi_)
        .def_rw("numTurns_", &ob::OwenStateSpace::PathType::numTurns_);

    nb::class_<ob::OwenStateSpace::StateType, ob::CompoundState>(m, "OwenStateType")
        .def("__getitem__", [](const ob::OwenStateSpace::StateType *s, unsigned int i) { return (*s)[i]; })
        .def("__setitem__", [](ob::OwenStateSpace::StateType *s, unsigned int i, double v) { (*s)[i] = v; })
        .def("getYaw", [](const ob::OwenStateSpace::StateType *s) { return s->yaw(); })
        .def("setYaw", [](ob::OwenStateSpace::StateType *s, double v) { s->yaw() = v; });

    nb::class_<ob::OwenStateSpace, ob::CompoundStateSpace>(m, "OwenStateSpace")
        .def(nb::init<double, double>(), nb::arg("turningRadius") = 1.0,
             nb::arg("maxPitch") = boost::math::double_constants::sixth_pi)
        .def("distance", &ob::OwenStateSpace::distance)
        .def("getPath", &ob::OwenStateSpace::getPath)
        .def("setBounds", &ob::OwenStateSpace::setBounds)
        .def("getBounds", &ob::OwenStateSpace::getBounds, nb::rv_policy::reference_internal)
        .def("getMinTurnRadius", &ob::OwenStateSpace::getMinTurnRadius)
        .def("getMaxPitch", &ob::OwenStateSpace::getMaxPitch)
        .def("setTolerance", &ob::OwenStateSpace::setTolerance)
        .def("getTolerance", &ob::OwenStateSpace::getTolerance)
        .def("isMetricSpace", &ob::OwenStateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::OwenStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::OwenStateSpace::hasSymmetricInterpolate)
        .def("validSegmentCount", &ob::OwenStateSpace::validSegmentCount)
        .def("allocState", &ob::OwenStateSpace::allocState)
        .def("registerProjections", &ob::OwenStateSpace::registerProjections);

    nb::class_<ob::Dubins3DMotionValidator<ob::OwenStateSpace>, ob::MotionValidator>(m, "OwenMotionValidator")
        .def(nb::init<ob::SpaceInformation *>());
}
