#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rlrt/BiRLRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRlrt_BiRLRT(nb::module_ &m)
{
    nb::class_<og::BiRLRT, ob::Planner>(m, "BiRLRT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::BiRLRT &self, nb::object what)
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
            "getPlannerData", [](const og::BiRLRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::BiRLRT::clear)
        .def("setup", &og::BiRLRT::setup)
        .def("setRange", &og::BiRLRT::setRange, nb::arg("distance"))
        .def("getRange", &og::BiRLRT::getRange)
        .def("setMaxDistanceNear", &og::BiRLRT::setMaxDistanceNear, nb::arg("dNear"))
        .def("getMaxDistanceNear", &og::BiRLRT::getMaxDistanceNear)
        .def("getKeepLast", &og::BiRLRT::getKeepLast)
        .def("setKeepLast", &og::BiRLRT::setKeepLast, nb::arg("keepLast"));
}
