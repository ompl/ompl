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
    // Create a submodule for SE3-specific bindings.
//     auto se3Sub = m.def_submodule("SE3", "Bindings for SE3StateSpace");

    // Bind the ScopedState for SE3StateSpace using your templated helper.
    auto scopedState = bind_scoped_state_template<ompl::base::SE3StateSpace>(
        m, "SE3ScopedState", "ScopedState for SE3StateSpace");
    
    // Optionally add convenience methods to the ScopedState binding.
    scopedState.def("getX", [](const ompl::base::ScopedState<ompl::base::SE3StateSpace>& self) {
        return self->getX();
    }, "Return the X coordinate");
    scopedState.def("getY", [](const ompl::base::ScopedState<ompl::base::SE3StateSpace>& self) {
        return self->getY();
    }, "Return the Y coordinate");
    scopedState.def("getZ", [](const ompl::base::ScopedState<ompl::base::SE3StateSpace>& self) {
        return self->getZ();
    }, "Return the Z coordinate");
    scopedState.def("setX", [](ompl::base::ScopedState<ompl::base::SE3StateSpace>& self, double x) {
        self->setX(x);
    }, "Set the X coordinate");
    scopedState.def("setY", [](ompl::base::ScopedState<ompl::base::SE3StateSpace>& self, double y) {
        self->setY(y);
    }, "Set the Y coordinate");
    scopedState.def("setZ", [](ompl::base::ScopedState<ompl::base::SE3StateSpace>& self, double z) {
        self->setZ(z);
    }, "Set the Z coordinate");
    scopedState.def("setXYZ", [](ompl::base::ScopedState<ompl::base::SE3StateSpace>& self,
                                   double x, double y, double z) {
        self->setXYZ(x, y, z);
    }, "Set the X, Y and Z coordinates");
    scopedState.def("rotation", [](ompl::base::ScopedState<ompl::base::SE3StateSpace>& self) -> ompl::base::SO3StateSpace::StateType & {
        return self->rotation();
    }, nb::rv_policy::reference_internal, "Return the rotation part");
    
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