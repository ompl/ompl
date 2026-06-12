#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARStwo(nb::module_ &m)
{
    nb::class_<og::SPARStwo, ob::Planner>(
        m, "SPARStwo",
        "Warning: SPARStwo may use internal threads for solution checking. "
        "Avoid Python StateValidityChecker trampolines when multithreading is enabled.")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("constructRoadmap",
             [](og::SPARStwo &self, nb::object what, bool stopOnMaxFail)
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
             [](og::SPARStwo &self, nb::object what)
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
            "getPlannerData", [](const og::SPARStwo &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SPARStwo::clear)
        .def("setup", &og::SPARStwo::setup)
        .def("setStretchFactor", &og::SPARStwo::setStretchFactor, nb::arg("t"))
        .def("setSparseDeltaFraction", &og::SPARStwo::setSparseDeltaFraction, nb::arg("D"))
        .def("setDenseDeltaFraction", &og::SPARStwo::setDenseDeltaFraction, nb::arg("d"))
        .def("setMaxFailures", &og::SPARStwo::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARStwo::getMaxFailures)
        .def("getDenseDeltaFraction", &og::SPARStwo::getDenseDeltaFraction)
        .def("getSparseDeltaFraction", &og::SPARStwo::getSparseDeltaFraction)
        .def("getStretchFactor", &og::SPARStwo::getStretchFactor)
        .def("clearQuery", &og::SPARStwo::clearQuery)
        .def("milestoneCount", &og::SPARStwo::milestoneCount);
}
