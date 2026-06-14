#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARS(nb::module_ &m)
{
    nb::class_<og::SPARS, ob::Planner>(m, "SPARS",
                                       "Warning: SPARS may use internal threads for solution checking. "
                                       "Avoid Python StateValidityChecker trampolines when multithreading is enabled.")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "constructRoadmap",
            [](og::SPARS &self, nb::object what, bool stopOnMaxFail)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                {
                    return self.constructRoadmap(nb::cast<ob::PlannerTerminationCondition>(what), stopOnMaxFail);
                }
                else if (nb::isinstance<double>(what))
                {
                    return self.constructRoadmap(ob::timedPlannerTerminationCondition(nb::cast<double>(what)),
                                                 stopOnMaxFail);
                }
                else
                {
                    throw nb::type_error(
                        "Invalid argument type for constructRoadmap. Expected PlannerTerminationCondition or double.");
                }
            },
            nb::arg("ptc"), nb::arg("stopOnMaxFail") = false)
        .def("solve",
             [](og::SPARS &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def(
            "getPlannerData", [](const og::SPARS &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SPARS::clear)
        .def("setup", &og::SPARS::setup)
        .def("setMaxFailures", &og::SPARS::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARS::getMaxFailures)
        .def("setDenseDeltaFraction", &og::SPARS::setDenseDeltaFraction, nb::arg("d"))
        .def("setSparseDeltaFraction", &og::SPARS::setSparseDeltaFraction, nb::arg("d"))
        .def("setStretchFactor", &og::SPARS::setStretchFactor, nb::arg("t"))
        .def("getDenseDeltaFraction", &og::SPARS::getDenseDeltaFraction)
        .def("getSparseDeltaFraction", &og::SPARS::getSparseDeltaFraction)
        .def("getStretchFactor", &og::SPARS::getStretchFactor)
        .def("clearQuery", &og::SPARS::clearQuery)
        .def("milestoneCount", &og::SPARS::milestoneCount)
        .def("guardCount", &og::SPARS::guardCount);
}
