#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/ABITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_ABITstar(nb::module_ &m)
{
    nb::class_<og::ABITstar, og::BITstar>(m, "ABITstar")
        .def(nb::init<const ob::SpaceInformationPtr &, const std::string &>(), nb::arg("si"),
             nb::arg("name") = "ABITstar")
        .def("solve",
             [](og::ABITstar &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 if (nb::isinstance<double>(what))
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 throw nb::type_error(
                     "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
             })
        .def(
            "getPlannerData", [](const og::ABITstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::ABITstar::clear)
        .def("setup", &og::ABITstar::setup)
        .def("setInitialInflationFactor", &og::ABITstar::setInitialInflationFactor, nb::arg("factor"))
        .def("getInitialInflationFactor", &og::ABITstar::getInitialInflationFactor)
        .def("setInflationScalingParameter", &og::ABITstar::setInflationScalingParameter, nb::arg("parameter"))
        .def("getInflationScalingParameter", &og::ABITstar::getInflationScalingParameter)
        .def("setTruncationScalingParameter", &og::ABITstar::setTruncationScalingParameter, nb::arg("parameter"))
        .def("getTruncationScalingParameter", &og::ABITstar::getTruncationScalingParameter)
        .def("getCurrentInflationFactor", &og::ABITstar::getCurrentInflationFactor)
        .def("getCurrentTruncationFactor", &og::ABITstar::getCurrentTruncationFactor);
}
