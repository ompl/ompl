#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/LazyPRMstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_LazyPRMstar(nb::module_ &m)
{
    nb::class_<og::LazyPRMstar, og::LazyPRM>(m, "LazyPRMstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(nb::init<const ob::PlannerData &>(), nb::arg("data"))
        .def("solve", ompl::binding::solveBinding<og::LazyPRMstar>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::LazyPRMstar>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::LazyPRMstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LazyPRM::clear)
        .def("setup", &og::LazyPRM::setup);
}
