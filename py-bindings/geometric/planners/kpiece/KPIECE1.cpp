#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersKpiece_KPIECE1(nb::module_ &m)
{
    nb::class_<og::KPIECE1, ob::Planner>(m, "KPIECE1")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::KPIECE1 &self, nb::object what) {
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
             [](const og::KPIECE1 &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        .def("clear", &og::KPIECE1::clear)
        .def("setup", &og::KPIECE1::setup)

        // Goal bias
        .def("setGoalBias", &og::KPIECE1::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::KPIECE1::getGoalBias)

        // Range
        .def("setRange", &og::KPIECE1::setRange, nb::arg("distance"))
        .def("getRange", &og::KPIECE1::getRange)

        // Border fraction
        .def("setBorderFraction", &og::KPIECE1::setBorderFraction, nb::arg("bp"))
        .def("getBorderFraction", &og::KPIECE1::getBorderFraction)

        // Min valid path fraction
        .def("setMinValidPathFraction", &og::KPIECE1::setMinValidPathFraction, nb::arg("fraction"))
        .def("getMinValidPathFraction", &og::KPIECE1::getMinValidPathFraction)

        // Failed expansion score factor
        .def("setFailedExpansionCellScoreFactor", &og::KPIECE1::setFailedExpansionCellScoreFactor, nb::arg("factor"))
        .def("getFailedExpansionCellScoreFactor", &og::KPIECE1::getFailedExpansionCellScoreFactor)

        // Projection evaluator
        .def("setProjectionEvaluator",
             static_cast<void (og::KPIECE1::*)(const ob::ProjectionEvaluatorPtr &)>(
                 &og::KPIECE1::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::KPIECE1::*)(const std::string &)>(&og::KPIECE1::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &og::KPIECE1::getProjectionEvaluator, nb::rv_policy::reference_internal);
}
