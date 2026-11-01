#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersKpiece_KPIECE1(nb::module_ &m)
{
    nb::class_<oc::KPIECE1, ob::Planner>(m, "KPIECE1")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("setGoalBias", &oc::KPIECE1::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::KPIECE1::getGoalBias)
        .def("setBorderFraction", &oc::KPIECE1::setBorderFraction, nb::arg("bp"))
        .def("getBorderFraction", &oc::KPIECE1::getBorderFraction)
        .def("setCellScoreFactor", &oc::KPIECE1::setCellScoreFactor, nb::arg("good"), nb::arg("bad"))
        .def("setBadCellScoreFactor", &oc::KPIECE1::setBadCellScoreFactor, nb::arg("bad"))
        .def("setGoodCellScoreFactor", &oc::KPIECE1::setGoodCellScoreFactor, nb::arg("good"))
        .def("getGoodCellScoreFactor", &oc::KPIECE1::getGoodCellScoreFactor)
        .def("getBadCellScoreFactor", &oc::KPIECE1::getBadCellScoreFactor)
        .def("setMaxCloseSamplesCount", &oc::KPIECE1::setMaxCloseSamplesCount, nb::arg("nCloseSamples"))
        .def("getMaxCloseSamplesCount", &oc::KPIECE1::getMaxCloseSamplesCount)
        .def("setProjectionEvaluator",
             nb::overload_cast<const ob::ProjectionEvaluatorPtr &>(&oc::KPIECE1::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator", nb::overload_cast<const std::string &>(&oc::KPIECE1::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &oc::KPIECE1::getProjectionEvaluator);
}
