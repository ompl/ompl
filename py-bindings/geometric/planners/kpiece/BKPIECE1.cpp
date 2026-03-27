#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersKpiece_BKPIECE1(nb::module_ &m)
{
    nb::class_<og::BKPIECE1, ob::Planner>(m, "BKPIECE1")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::BKPIECE1 &self, nb::object what) {
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
             [](const og::BKPIECE1 &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        .def("clear", &og::BKPIECE1::clear)
        .def("setup", &og::BKPIECE1::setup)

        // Range
        .def("setRange", &og::BKPIECE1::setRange, nb::arg("distance"))
        .def("getRange", &og::BKPIECE1::getRange)

        // Border fraction
        .def("setBorderFraction", &og::BKPIECE1::setBorderFraction, nb::arg("bp"))
        .def("getBorderFraction", &og::BKPIECE1::getBorderFraction)

        // Min valid path fraction
        .def("setMinValidPathFraction", &og::BKPIECE1::setMinValidPathFraction, nb::arg("fraction"))
        .def("getMinValidPathFraction", &og::BKPIECE1::getMinValidPathFraction)

        // Failed expansion score factor
        .def("setFailedExpansionCellScoreFactor", &og::BKPIECE1::setFailedExpansionCellScoreFactor, nb::arg("factor"))
        .def("getFailedExpansionCellScoreFactor", &og::BKPIECE1::getFailedExpansionCellScoreFactor)

        // Projection evaluator
        .def("setProjectionEvaluator",
             static_cast<void (og::BKPIECE1::*)(const ob::ProjectionEvaluatorPtr &)>(
                 &og::BKPIECE1::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::BKPIECE1::*)(const std::string &)>(&og::BKPIECE1::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &og::BKPIECE1::getProjectionEvaluator, nb::rv_policy::reference_internal);
}
