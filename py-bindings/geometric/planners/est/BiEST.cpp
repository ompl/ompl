#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersEst_BiEST(nb::module_ &m)
{
    nb::class_<og::BiEST, ob::Planner>(m, "BiEST")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::BiEST>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::BiEST>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::BiEST &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::BiEST::clear)
        .def("setup", &og::BiEST::setup)
        .def("setRange", &og::BiEST::setRange, nb::arg("distance"))
        .def("getRange", &og::BiEST::getRange);
}
