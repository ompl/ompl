#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/EIRMstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_EIRMstar(nb::module_ &m)
{
    nb::class_<og::EIRMstar, og::EITstar>(m, "EIRMstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "solve",
            [](og::EIRMstar &self, nb::object what)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                    return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                if (nb::isinstance<double>(what))
                    return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                throw nb::type_error(
                    "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
            })
        .def(
            "getPlannerData", [](const og::EIRMstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::EIRMstar::clear)
        .def("setup", &og::EIRMstar::setup)
        .def("setStartGoalPruningThreshold", &og::EIRMstar::setStartGoalPruningThreshold, nb::arg("threshold"))
        .def("getStartGoalPruningThreshold", &og::EIRMstar::getStartGoalPruningThreshold);
}
