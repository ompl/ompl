#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/planners/sst/SST.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSst_SST(nb::module_ &m)
{
    nb::class_<oc::SST, ob::Planner>(m, "SST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("setup", &oc::SST::setup)
        .def("solve", &oc::SST::solve, nb::arg("terminationCondition"))
        .def("getPlannerData", [](const oc::SST &planner, ob::PlannerData &data) { planner.getPlannerData(data); },
             nb::arg("data"))
        .def("clear", &oc::SST::clear)
        .def("setGoalBias", &oc::SST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::SST::getGoalBias)
        .def("setSelectionRadius", &oc::SST::setSelectionRadius, nb::arg("selectionRadius"))
        .def("getSelectionRadius", &oc::SST::getSelectionRadius)
        .def("setPruningRadius", &oc::SST::setPruningRadius, nb::arg("pruningRadius"))
        .def("getPruningRadius", &oc::SST::getPruningRadius);
}
