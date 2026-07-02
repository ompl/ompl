#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/planners/sst/SSTstar.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

void ompl::binding::control::initPlannersSst_SSTstar(nb::module_ &m)
{
    nb::class_<oc::SSTstar, ob::Planner>(m, "SSTstar")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("setup", &oc::SSTstar::setup)
        .def("solve", &oc::SSTstar::solve, nb::arg("terminationCondition"))
        .def(
            "getPlannerData", [](const oc::SSTstar &planner, ob::PlannerData &data) { planner.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &oc::SSTstar::clear)
        .def("setGoalBias", &oc::SSTstar::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::SSTstar::getGoalBias)
        .def("setSelectionRadius", &oc::SSTstar::setSelectionRadius, nb::arg("selectionRadius"))
        .def("getSelectionRadius", &oc::SSTstar::getSelectionRadius)
        .def("setPruningRadius", &oc::SSTstar::setPruningRadius, nb::arg("pruningRadius"))
        .def("getPruningRadius", &oc::SSTstar::getPruningRadius)
        .def("setInitialIteration", &oc::SSTstar::setInitialIteration, nb::arg("initialN"))
        .def("getInitialIteration", &oc::SSTstar::getInitialIteration)
        .def("setShrinkFactor", &oc::SSTstar::setShrinkFactor, nb::arg("shrinkFactor"))
        .def("getShrinkFactor", &oc::SSTstar::getShrinkFactor);
}
