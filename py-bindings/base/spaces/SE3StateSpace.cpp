#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/base/spaces/SE3StateSpace.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SE3StateSpace(nb::module_ &m)
{
    // Bind the inner state type for SE3StateSpace.
    nb::class_<ompl::base::SE3StateSpace::StateType, ompl::base::State>(m, "SE3State")
        .def("getX", &ompl::base::SE3StateSpace::StateType::getX)
        .def("getY", &ompl::base::SE3StateSpace::StateType::getY)
        .def("getZ", &ompl::base::SE3StateSpace::StateType::getZ)
        .def("setX", &ompl::base::SE3StateSpace::StateType::setX)
        .def("setY", &ompl::base::SE3StateSpace::StateType::setY)
        .def("setZ", &ompl::base::SE3StateSpace::StateType::setZ)
        .def("setXYZ", &ompl::base::SE3StateSpace::StateType::setXYZ)
        .def("rotation", nb::overload_cast<>(
              &ompl::base::SE3StateSpace::StateType::rotation), nb::rv_policy::reference_internal);
    
    // Bind the SE3StateSpace class.
    nb::class_<ompl::base::SE3StateSpace,
               ompl::base::CompoundStateSpace>(m, "SE3StateSpace")
        .def(nb::init<>())
        .def("setBounds", &ompl::base::SE3StateSpace::setBounds)
        .def("getBounds", &ompl::base::SE3StateSpace::getBounds)
        .def("allocState", &ompl::base::SE3StateSpace::allocState)
        .def("registerProjections", &ompl::base::SE3StateSpace::registerProjections);
}