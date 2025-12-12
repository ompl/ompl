#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "../init.hh"
#include "common.hh"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SE2StateSpace(nb::module_& m)
{
    nb::class_<ompl::base::SE2StateSpace::StateType, ompl::base::State>(m, "SE2StateType")
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
        .def("registerProjections", &ompl::base::SE2StateSpace::registerProjections);
}
