#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/optional.h>
#include "ompl/base/spaces/VanaOwenStateSpace.h"
#include "ompl/base/spaces/Dubins3DMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_VanaOwenStateSpace(nb::module_ &m)
{
    nb::enum_<ob::VanaOwenStateSpace::PathCategory>(m, "VanaOwenPathCategory")
        .value("LOW_ALTITUDE", ob::VanaOwenStateSpace::PathCategory::LOW_ALTITUDE)
        .value("MEDIUM_ALTITUDE", ob::VanaOwenStateSpace::PathCategory::MEDIUM_ALTITUDE)
        .value("HIGH_ALTITUDE", ob::VanaOwenStateSpace::PathCategory::HIGH_ALTITUDE)
        .value("UNKNOWN", ob::VanaOwenStateSpace::PathCategory::UNKNOWN);

    nb::class_<ob::VanaOwenStateSpace::PathType>(m, "VanaOwenPathType")
        .def("length", &ob::VanaOwenStateSpace::PathType::length)
        .def("category", &ob::VanaOwenStateSpace::PathType::category)
        .def_rw("horizontalRadius_", &ob::VanaOwenStateSpace::PathType::horizontalRadius_)
        .def_rw("verticalRadius_", &ob::VanaOwenStateSpace::PathType::verticalRadius_)
        .def_rw("deltaZ_", &ob::VanaOwenStateSpace::PathType::deltaZ_)
        .def_rw("phi_", &ob::VanaOwenStateSpace::PathType::phi_)
        .def_rw("numTurns_", &ob::VanaOwenStateSpace::PathType::numTurns_);

    nb::class_<ob::VanaOwenStateSpace::StateType, ob::CompoundState>(m, "VanaOwenStateType")
        .def("__getitem__", [](const ob::VanaOwenStateSpace::StateType *s, unsigned int i) { return (*s)[i]; })
        .def("__setitem__", [](ob::VanaOwenStateSpace::StateType *s, unsigned int i, double v) { (*s)[i] = v; })
        .def("getYaw", [](const ob::VanaOwenStateSpace::StateType *s) { return s->yaw(); })
        .def("setYaw", [](ob::VanaOwenStateSpace::StateType *s, double v) { s->yaw() = v; })
        .def("getPitch", [](const ob::VanaOwenStateSpace::StateType *s) { return s->pitch(); })
        .def("setPitch", [](ob::VanaOwenStateSpace::StateType *s, double v) { s->pitch() = v; });

    nb::class_<ob::VanaOwenStateSpace, ob::CompoundStateSpace>(m, "VanaOwenStateSpace")
        .def(nb::init<double, double>(), nb::arg("turningRadius") = 1.0,
             nb::arg("maxPitch") = boost::math::double_constants::sixth_pi)
        .def(nb::init<double, std::pair<double, double>>(), nb::arg("turningRadius"), nb::arg("pitchRange"))
        .def("distance", &ob::VanaOwenStateSpace::distance)
        .def("getPath", &ob::VanaOwenStateSpace::getPath)
        .def("setBounds", &ob::VanaOwenStateSpace::setBounds)
        .def("getBounds", &ob::VanaOwenStateSpace::getBounds, nb::rv_policy::reference_internal)
        .def("setTolerance", &ob::VanaOwenStateSpace::setTolerance)
        .def("getTolerance", &ob::VanaOwenStateSpace::getTolerance)
        .def("isMetricSpace", &ob::VanaOwenStateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::VanaOwenStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::VanaOwenStateSpace::hasSymmetricInterpolate)
        .def("validSegmentCount", &ob::VanaOwenStateSpace::validSegmentCount)
        .def("allocState", &ob::VanaOwenStateSpace::allocState)
        .def("registerProjections", &ob::VanaOwenStateSpace::registerProjections);

    nb::class_<ob::Dubins3DMotionValidator<ob::VanaOwenStateSpace>, ob::MotionValidator>(m, "VanaOwenMotionValidator")
        .def(nb::init<ob::SpaceInformation *>());
}
