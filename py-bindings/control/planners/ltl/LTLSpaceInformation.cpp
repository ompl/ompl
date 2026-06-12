#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/ltl/LTLSpaceInformation.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersLtl_LTLSpaceInformation(nb::module_ &m)
{
    nb::class_<oc::LTLSpaceInformation, oc::SpaceInformation>(m, "LTLSpaceInformation")
        .def(nb::init<const oc::SpaceInformationPtr &, const oc::ProductGraphPtr &>(), nb::arg("si"),
             nb::arg("prod"))
        .def("setup", &oc::LTLSpaceInformation::setup)
        .def("getProductGraph", &oc::LTLSpaceInformation::getProductGraph, nb::rv_policy::reference_internal)
        .def("getLowSpace", &oc::LTLSpaceInformation::getLowSpace, nb::rv_policy::reference_internal)
        .def("getFullState", &oc::LTLSpaceInformation::getFullState, nb::arg("low"), nb::arg("full"))
        .def("getLowLevelState",
             nb::overload_cast<ob::State *>(&oc::LTLSpaceInformation::getLowLevelState), nb::arg("s"),
             nb::rv_policy::reference_internal)
        .def("getLowLevelState",
             nb::overload_cast<const ob::State *>(&oc::LTLSpaceInformation::getLowLevelState),
             nb::arg("s"), nb::rv_policy::reference_internal)
        .def("getProdGraphState", &oc::LTLSpaceInformation::getProdGraphState, nb::arg("s"),
             nb::rv_policy::reference_internal);
}
