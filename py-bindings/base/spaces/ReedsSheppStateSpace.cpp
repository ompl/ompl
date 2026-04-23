#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/DubinsMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_ReedsSheppStateSpace(nb::module_ &m)
{
    nb::enum_<ob::ReedsSheppStateSpace::ReedsSheppPathSegmentType>(m, "ReedsSheppPathSegmentType")
        .value("RS_NOP", ob::ReedsSheppStateSpace::RS_NOP)
        .value("RS_LEFT", ob::ReedsSheppStateSpace::RS_LEFT)
        .value("RS_STRAIGHT", ob::ReedsSheppStateSpace::RS_STRAIGHT)
        .value("RS_RIGHT", ob::ReedsSheppStateSpace::RS_RIGHT);

    nb::class_<ob::ReedsSheppStateSpace::PathType>(m, "ReedsSheppPathType")
        .def("length", &ob::ReedsSheppStateSpace::PathType::length)
        .def_ro("totalLength_", &ob::ReedsSheppStateSpace::PathType::totalLength_);

    nb::class_<ob::ReedsSheppStateSpace, ob::SE2StateSpace>(m, "ReedsSheppStateSpace")
        .def(nb::init<double>(), nb::arg("turningRadius") = 1.0)
        .def("distance", &ob::ReedsSheppStateSpace::distance)
        .def("getPath", &ob::ReedsSheppStateSpace::getPath)
        .def("validSegmentCount", &ob::ReedsSheppStateSpace::validSegmentCount)
        .def("allocState", &ob::ReedsSheppStateSpace::allocState);

    nb::class_<ob::DubinsMotionValidator<ob::ReedsSheppStateSpace>, ob::MotionValidator>(m, "ReedsSheppMotionValidator")
        .def(nb::init<ob::SpaceInformation *>());
}
