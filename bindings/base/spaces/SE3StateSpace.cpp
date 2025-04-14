#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/base/spaces/SE3StateSpace.h"
#include "../init.hh"
#include "common.hh"  // Assumes bind_scoped_state_template is defined here

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SE3StateSpace(nb::module_ &m)
{
    // Bind the inner state type for SE3StateSpace.
    nb::class_<ompl::base::SE3StateSpace::StateType, ompl::base::State>(m, "SE3State")
        .def("getX", &ompl::base::SE3StateSpace::StateType::getX,
             "Return the X coordinate")
        .def("getY", &ompl::base::SE3StateSpace::StateType::getY,
             "Return the Y coordinate")
        .def("getZ", &ompl::base::SE3StateSpace::StateType::getZ,
             "Return the Z coordinate")
        .def("setX", &ompl::base::SE3StateSpace::StateType::setX,
             "Set the X coordinate")
        .def("setY", &ompl::base::SE3StateSpace::StateType::setY,
             "Set the Y coordinate")
        .def("setZ", &ompl::base::SE3StateSpace::StateType::setZ,
             "Set the Z coordinate")
        .def("setXYZ", &ompl::base::SE3StateSpace::StateType::setXYZ,
             "Set the X, Y and Z coordinates")
        .def("rotation", nb::overload_cast<>(
              &ompl::base::SE3StateSpace::StateType::rotation), nb::rv_policy::reference_internal,
             "Return the rotation part");
    
    // Bind the SE3StateSpace class.
    nb::class_<ompl::base::SE3StateSpace,
               ompl::base::CompoundStateSpace>(m, "SE3StateSpace")
        .def(nb::init<>(), "Default constructor for SE3StateSpace")
        .def("setBounds", &ompl::base::SE3StateSpace::setBounds,
             "Set the bounds of the translational (RealVector) component")
        .def("getBounds", &ompl::base::SE3StateSpace::getBounds,
             "Return the bounds of the translational component")
        .def("allocState", &ompl::base::SE3StateSpace::allocState,
             "Allocate a state for the SE3 state space")
        .def("registerProjections", &ompl::base::SE3StateSpace::registerProjections,
             "Register projections for the state space");
}