#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/pdst/PDST.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPdst_PDST(nb::module_ &m)
{
    nb::class_<og::PDST, ob::Planner>(m, "PDST")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::PDST>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::PDST>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::PDST &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::PDST::clear)
        .def("setup", &og::PDST::setup)
        .def("setProjectionEvaluator",
             static_cast<void (og::PDST::*)(const ob::ProjectionEvaluatorPtr &)>(&og::PDST::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator",
             static_cast<void (og::PDST::*)(const std::string &)>(&og::PDST::setProjectionEvaluator), nb::arg("name"))
        .def("getProjectionEvaluator", &og::PDST::getProjectionEvaluator, nb::rv_policy::reference_internal)
        .def("setGoalBias", &og::PDST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::PDST::getGoalBias);
}
