#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersSbl_SBL(nb::module_ &m)
{
    nb::class_<og::SBL, ob::Planner>(m, "SBL")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::SBL>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::SBL>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::SBL &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SBL::clear)
        .def("setup", &og::SBL::setup)
        .def("setProjectionEvaluator",
             static_cast<void (og::SBL::*)(const ob::ProjectionEvaluatorPtr &)>(&og::SBL::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::SBL::*)(const std::string &)>(&og::SBL::setProjectionEvaluator), nb::arg("name"))
        .def("getProjectionEvaluator", &og::SBL::getProjectionEvaluator, nb::rv_policy::reference_internal)
        .def("setRange", &og::SBL::setRange, nb::arg("distance"))
        .def("getRange", &og::SBL::getRange);
}
