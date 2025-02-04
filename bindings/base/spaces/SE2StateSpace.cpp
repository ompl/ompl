#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSpace.h"
#include "ompl/base/State.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "../init.hh"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SE2StateSpace(nb::module_& m)
{
    nb::class_<ompl::base::SE2StateSpace::StateType>(m, "SE2StateType")
        .def("getX", &ompl::base::SE2StateSpace::StateType::getX)
        .def("getY", &ompl::base::SE2StateSpace::StateType::getY)
        .def("getYaw", &ompl::base::SE2StateSpace::StateType::getYaw)
        .def("setX", &ompl::base::SE2StateSpace::StateType::setX)
        .def("setY", &ompl::base::SE2StateSpace::StateType::setY)
        .def("setXY", &ompl::base::SE2StateSpace::StateType::setXY)
        .def("setYaw", &ompl::base::SE2StateSpace::StateType::setYaw);

    auto se2Sub = m.def_submodule("se2");
    nb::class_<ompl::base::ScopedState<ompl::base::SE2StateSpace>>(se2Sub, "State")
        .def(nb::init<const std::shared_ptr<ompl::base::SpaceInformation>&>())
        .def(nb::init<std::shared_ptr<ompl::base::StateSpace>>())
        .def("random", &ompl::base::ScopedState<ompl::base::SE2StateSpace>::random)
        .def("getX", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self) {return self->getX();})
        .def("getY", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self) {return self->getY();})
        .def("getYaw", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self) {return self->getYaw();})
        .def("setX", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self, double x) {self->setX(x);})
        .def("setY", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self, double y) {self->setY(y);})
        .def("setXY", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self, double x, double y) {self->setXY(x, y);})
        .def("setYaw", [](ompl::base::ScopedState<ompl::base::SE2StateSpace>& self, double yaw) {self->setYaw(yaw);});
        
    nb::class_<ompl::base::SE2StateSpace, ompl::base::CompoundStateSpace>(m, "_SE2StateSpace")
        .def(nb::init<>())
        .def("setBounds", &ompl::base::SE2StateSpace::setBounds)
        .def("getBounds", &ompl::base::SE2StateSpace::getBounds)
        .def("allocState", &ompl::base::SE2StateSpace::allocState)
        .def("freeState", &ompl::base::SE2StateSpace::freeState)
        .def("registerProjections", &ompl::base::SE2StateSpace::registerProjections);
    m.def("SE2StateSpace", []() { return std::make_shared<ompl::base::SE2StateSpace>(); }, nb::rv_policy::reference);
}
