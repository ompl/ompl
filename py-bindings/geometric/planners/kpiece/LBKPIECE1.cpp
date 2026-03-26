#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersKpiece_LBKPIECE1(nb::module_ &m)
{
    nb::class_<og::LBKPIECE1, ob::Planner>(m, "LBKPIECE1")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::LBKPIECE1 &self, nb::object what) {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what)) {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 } else if (nb::isinstance<double>(what)) {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 } else {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })

        .def("getPlannerData",
             [](const og::LBKPIECE1 &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        .def("clear", &og::LBKPIECE1::clear)
        .def("setup", &og::LBKPIECE1::setup)

        // Range
        .def("setRange", &og::LBKPIECE1::setRange, nb::arg("distance"))
        .def("getRange", &og::LBKPIECE1::getRange)

        // Border fraction
        .def("setBorderFraction", &og::LBKPIECE1::setBorderFraction, nb::arg("bp"))
        .def("getBorderFraction", &og::LBKPIECE1::getBorderFraction)

        // Min valid path fraction
        .def("setMinValidPathFraction", &og::LBKPIECE1::setMinValidPathFraction, nb::arg("fraction"))
        .def("getMinValidPathFraction", &og::LBKPIECE1::getMinValidPathFraction)

        // Projection evaluator
        .def("setProjectionEvaluator",
             static_cast<void (og::LBKPIECE1::*)(const ob::ProjectionEvaluatorPtr &)>(
                 &og::LBKPIECE1::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::LBKPIECE1::*)(const std::string &)>(&og::LBKPIECE1::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &og::LBKPIECE1::getProjectionEvaluator, nb::rv_policy::reference_internal);
}
