#include <nanobind/nanobind.h>
#include "ompl/base/spaces/SE2StateSpace.h"

namespace nb = nanobind;

void initSE2StateSpace(nb::module_& m) {
    // Bind StateType
    nb::class_<ompl::base::SE2StateSpace::StateType, ompl::base::CompoundStateSpace::StateType>(m, "SE2StateType")
        .def(nb::init<>())
        .def("GetX", &ompl::base::SE2StateSpace::StateType::getX)
        .def("GetY", &ompl::base::SE2StateSpace::StateType::getY)
        .def("GetYaw", &ompl::base::SE2StateSpace::StateType::getYaw)
        .def("SetX", &ompl::base::SE2StateSpace::StateType::setX)
        .def("SetY", &ompl::base::SE2StateSpace::StateType::setY)
        .def("SetXY", &ompl::base::SE2StateSpace::StateType::setXY)
        .def("SetYaw", &ompl::base::SE2StateSpace::StateType::setYaw);

    // Bind SE2StateSpace
    nb::class_<ompl::base::SE2StateSpace, ompl::base::CompoundStateSpace>(m, "SE2StateSpace")
        .def(nb::init<>())
        .def("SetBounds", &ompl::base::SE2StateSpace::setBounds)
        .def("GetBounds", &ompl::base::SE2StateSpace::getBounds)
        .def("AllocState", &ompl::base::SE2StateSpace::allocState)
        .def("FreeState", &ompl::base::SE2StateSpace::freeState)
        .def("RegisterProjections", &ompl::base::SE2StateSpace::registerProjections);
}