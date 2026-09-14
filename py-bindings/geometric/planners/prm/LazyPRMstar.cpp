#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/LazyPRMstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_LazyPRMstar(nb::module_ &m)
{
    nb::class_<og::LazyPRMstar, og::LazyPRM>(
        m, "LazyPRMstar",
        "Warning: LazyPRMstar may use internal threads for solution checking. "
        "Avoid Python StateValidityChecker trampolines when multithreading is enabled.")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(nb::init<const ob::PlannerData &>(), nb::arg("data"))
        .def("solve",
             [](og::LazyPRMstar &self, nb::object what)
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
            "getPlannerData", [](const og::LazyPRMstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LazyPRM::clear)
        .def("setup", &og::LazyPRM::setup);
}
