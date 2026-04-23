#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/optional.h>
#include "ompl/base/spaces/VanaStateSpace.h"
#include "ompl/base/spaces/Dubins3DMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_VanaStateSpace(nb::module_ &m)
{
    nb::class_<ob::VanaStateSpace::PathType>(m, "VanaPathType")
        .def("length", &ob::VanaStateSpace::PathType::length)
        .def_rw("horizontalRadius_", &ob::VanaStateSpace::PathType::horizontalRadius_)
        .def_rw("verticalRadius_", &ob::VanaStateSpace::PathType::verticalRadius_);

    nb::class_<ob::VanaStateSpace::StateType, ob::CompoundState>(m, "VanaStateType")
        .def("__getitem__", [](const ob::VanaStateSpace::StateType *s, unsigned int i) { return (*s)[i]; })
        .def("__setitem__", [](ob::VanaStateSpace::StateType *s, unsigned int i, double v) { (*s)[i] = v; })
        .def("getYaw", [](const ob::VanaStateSpace::StateType *s) { return s->yaw(); })
        .def("setYaw", [](ob::VanaStateSpace::StateType *s, double v) { s->yaw() = v; })
        .def("getPitch", [](const ob::VanaStateSpace::StateType *s) { return s->pitch(); })
        .def("setPitch", [](ob::VanaStateSpace::StateType *s, double v) { s->pitch() = v; });

    nb::class_<ob::VanaStateSpace, ob::CompoundStateSpace>(m, "VanaStateSpace")
        .def(nb::init<double, double>(), nb::arg("turningRadius") = 1.0,
             nb::arg("maxPitch") = boost::math::double_constants::sixth_pi)
        .def(nb::init<double, std::pair<double, double>>(), nb::arg("turningRadius"), nb::arg("pitchRange"))
        .def("distance", &ob::VanaStateSpace::distance)
        .def("getPath", &ob::VanaStateSpace::getPath)
        .def("setBounds", &ob::VanaStateSpace::setBounds)
        .def("getBounds", &ob::VanaStateSpace::getBounds, nb::rv_policy::reference_internal)
        .def("setTolerance", &ob::VanaStateSpace::setTolerance)
        .def("getTolerance", &ob::VanaStateSpace::getTolerance)
        .def("isMetricSpace", &ob::VanaStateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::VanaStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::VanaStateSpace::hasSymmetricInterpolate)
        .def("validSegmentCount", &ob::VanaStateSpace::validSegmentCount)
        .def("allocState", &ob::VanaStateSpace::allocState)
        .def("registerProjections", &ob::VanaStateSpace::registerProjections);

    nb::class_<ob::Dubins3DMotionValidator<ob::VanaStateSpace>, ob::MotionValidator>(m, "VanaMotionValidator")
        .def(nb::init<ob::SpaceInformation *>());
}
