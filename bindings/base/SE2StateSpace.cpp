#include <nanobind/nanobind.h>
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/StateSpace.h"

namespace nb = nanobind;

void initSE2StateSpace(nb::module_& m) {
    nb::class_<ompl::base::SE2StateSpace::StateType, ompl::base::CompoundStateSpace::StateType>(m, "SE2StateType")
        .def(nb::init<>())
        .def("getX", &ompl::base::SE2StateSpace::StateType::getX)
        .def("getY", &ompl::base::SE2StateSpace::StateType::getY)
        .def("getYaw", &ompl::base::SE2StateSpace::StateType::getYaw)
        .def("setX", &ompl::base::SE2StateSpace::StateType::setX)
        .def("setY", &ompl::base::SE2StateSpace::StateType::setY)
        .def("setXY", &ompl::base::SE2StateSpace::StateType::setXY)
        .def("setYaw", &ompl::base::SE2StateSpace::StateType::setYaw);

    nb::class_<ompl::base::SE2StateSpace, ompl::base::CompoundStateSpace>(m, "SE2StateSpace")
        .def(nb::init<>())
        .def("setBounds", &ompl::base::SE2StateSpace::setBounds)
        .def("getBounds", &ompl::base::SE2StateSpace::getBounds)
        .def("allocState", &ompl::base::SE2StateSpace::allocState)
        .def("freeState", &ompl::base::SE2StateSpace::freeState)
        .def("registerProjections", &ompl::base::SE2StateSpace::registerProjections);
}