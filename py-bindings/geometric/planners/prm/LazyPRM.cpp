#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_LazyPRM(nb::module_ &m)
{
    nb::class_<og::LazyPRM, ob::Planner>(
        m, "LazyPRM",
        "Warning: LazyPRM may use internal threads for solution checking. "
        "Avoid Python StateValidityChecker trampolines when multithreading is enabled.")
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("starStrategy") = false)
        .def(nb::init<const ob::PlannerData &, bool>(), nb::arg("data"), nb::arg("starStrategy") = false)
        .def("solve",
             [](og::LazyPRM &self, nb::object what)
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
            "getPlannerData", [](const og::LazyPRM &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LazyPRM::clear)
        .def("setup", &og::LazyPRM::setup)
        .def("setRange", &og::LazyPRM::setRange, nb::arg("distance"))
        .def("getRange", &og::LazyPRM::getRange)
        .def("setDefaultConnectionStrategy", &og::LazyPRM::setDefaultConnectionStrategy)
        .def("setMaxNearestNeighbors", &og::LazyPRM::setMaxNearestNeighbors, nb::arg("k"))
        .def("clearQuery", &og::LazyPRM::clearQuery)
        .def("clearValidity", &og::LazyPRM::clearValidity)
        .def("milestoneCount", &og::LazyPRM::milestoneCount)
        .def("edgeCount", &og::LazyPRM::edgeCount);
}
