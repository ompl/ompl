#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/AORRTC.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_AORRTC(nb::module_ &m)
{
    nb::class_<og::AORRTC, ob::Planner>(m, "AORRTC")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::AORRTC &self, nb::object what) {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what)) {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 } else if (nb::isinstance<double>(what)) {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 } else {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })

        // getPlannerData
        .def("getPlannerData",
             [](const og::AORRTC &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        // clear / setup
        .def("clear", &og::AORRTC::clear)
        .def("setup", &og::AORRTC::setup)

        // Range
        .def("setRange", &og::AORRTC::setRange, nb::arg("distance"))
        .def("getRange", &og::AORRTC::getRange)

        // Best cost
        .def("bestCost", &og::AORRTC::bestCost);
}

