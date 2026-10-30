#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/control/planners/rrt/AORRT.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/PlannerData.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersRrt_AORRT(nb::module_ &m)
{
    nb::class_<oc::AORRT, ob::Planner>(m, "AORRT")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", &oc::AORRT::solve, nb::arg("terminationCondition"))
        .def("solveOnce", &oc::AORRT::solveOnce, nb::arg("terminationCondition"))
        .def("clear", &oc::AORRT::clear)
        .def("setGoalBias", &oc::AORRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::AORRT::getGoalBias)
        .def("setCostWeight", &oc::AORRT::setCostWeight, nb::arg("weight"))
        .def("getCostWeight", &oc::AORRT::getCostWeight)
        .def("setIntermediateStates", &oc::AORRT::setIntermediateStates, nb::arg("addIntermediateStates"))
        .def("getIntermediateStates", &oc::AORRT::getIntermediateStates)
        .def(
            "getPlannerData", [](const oc::AORRT &planner, ob::PlannerData &data) { planner.getPlannerData(data); },
            nb::arg("data"))
        .def("setup", &oc::AORRT::setup)
        .def("getBestCost", &oc::AORRT::getBestCost);
}
