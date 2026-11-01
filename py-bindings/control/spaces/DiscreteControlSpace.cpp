#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <iostream>
#include <sstream>

#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "../OwnedControl.h"
#include "../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initSpaces_DiscreteControlSpace(nb::module_ &m)
{
    nb::class_<oc::DiscreteControlSpace::ControlType, oc::Control>(m, "DiscreteControlType")
        .def_rw("value", &oc::DiscreteControlSpace::ControlType::value);

    nb::class_<oc::DiscreteControlSpace, oc::ControlSpace>(m, "DiscreteControlSpace")
        .def(nb::init<const ob::StateSpacePtr &, int, int>(), nb::arg("stateSpace"), nb::arg("lowerBound"),
             nb::arg("upperBound"))
        .def("getDimension", &oc::DiscreteControlSpace::getDimension)
        .def("getControlCount", &oc::DiscreteControlSpace::getControlCount)
        .def("getLowerBound", &oc::DiscreteControlSpace::getLowerBound)
        .def("getUpperBound", &oc::DiscreteControlSpace::getUpperBound)
        .def("setBounds", &oc::DiscreteControlSpace::setBounds, nb::arg("lowerBound"), nb::arg("upperBound"))
        .def("allocControl",
             [](nb::pointer_and_handle<oc::DiscreteControlSpace> self)
             {
                 auto space = nb::cast<oc::ControlSpacePtr>(self.h);
                 return ownedControl(space, space->allocControl());
             })
        .def("copyControl", &oc::DiscreteControlSpace::copyControl, nb::arg("destination"), nb::arg("source"))
        .def("equalControls", &oc::DiscreteControlSpace::equalControls, nb::arg("control1"), nb::arg("control2"))
        .def("nullControl", &oc::DiscreteControlSpace::nullControl, nb::arg("control"))
        .def("allocDefaultControlSampler", &oc::DiscreteControlSpace::allocDefaultControlSampler)
        .def(
            "printControl",
            [](const oc::DiscreteControlSpace &space, const oc::Control *ctrl)
            {
                std::ostringstream oss;
                space.printControl(ctrl, oss);
                return oss.str();
            },
            nb::arg("control"))
        .def("printSettings", [](const oc::DiscreteControlSpace &space) { space.printSettings(std::cout); })
        .def("__repr__",
             [](const oc::DiscreteControlSpace &space)
             {
                 std::ostringstream oss;
                 space.printSettings(oss);
                 return oss.str();
             })
        .def("setup", &oc::DiscreteControlSpace::setup);
}
