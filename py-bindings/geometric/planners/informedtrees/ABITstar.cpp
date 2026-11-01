#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/ABITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_ABITstar(nb::module_ &m)
{
    nb::class_<og::ABITstar, og::BITstar>(m, "ABITstar")
        .def(nb::init<const ob::SpaceInformationPtr &, const std::string &>(), nb::arg("si"),
             nb::arg("name") = "ABITstar")
        .def("solve", ompl::binding::solveBinding<og::ABITstar>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::ABITstar>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
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
