#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/sst/SST.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersSst_SST(nb::module_ &m)
{
    nb::class_<og::SST, ob::Planner>(m, "SST")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::SST>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::SST>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::SST &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SST::clear)
        .def("setup", &og::SST::setup)
        .def("setGoalBias", &og::SST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::SST::getGoalBias)
        .def("setRange", &og::SST::setRange, nb::arg("distance"))
        .def("getRange", &og::SST::getRange)
        .def("setSelectionRadius", &og::SST::setSelectionRadius, nb::arg("selectionRadius"))
        .def("getSelectionRadius", &og::SST::getSelectionRadius)
        .def("setPruningRadius", &og::SST::setPruningRadius, nb::arg("pruningRadius"))
        .def("getPruningRadius", &og::SST::getPruningRadius);
}
