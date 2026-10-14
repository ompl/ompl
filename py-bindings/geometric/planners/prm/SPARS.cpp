#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARS(nb::module_ &m)
{
    nb::class_<og::SPARS, ob::Planner>(m, "SPARS")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "constructRoadmap",
            [](og::SPARS &self, nb::handle what, bool stopOnMaxFail)
            {
                const ob::PlannerTerminationCondition ptc = ompl::binding::toTerminationCondition(what);
                nb::gil_scoped_release release;
                self.constructRoadmap(ptc, stopOnMaxFail);
            },
            nb::arg("ptc"), nb::arg("stopOnMaxFail") = false)
        .def("solve", ompl::binding::solveBinding<og::SPARS>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::SPARS>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::SPARS &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SPARS::clear)
        .def("setup", &og::SPARS::setup)
        .def("setMaxFailures", &og::SPARS::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARS::getMaxFailures)
        .def("setDenseDeltaFraction", &og::SPARS::setDenseDeltaFraction, nb::arg("d"))
        .def("setSparseDeltaFraction", &og::SPARS::setSparseDeltaFraction, nb::arg("d"))
        .def("setStretchFactor", &og::SPARS::setStretchFactor, nb::arg("t"))
        .def("getDenseDeltaFraction", &og::SPARS::getDenseDeltaFraction)
        .def("getSparseDeltaFraction", &og::SPARS::getSparseDeltaFraction)
        .def("getStretchFactor", &og::SPARS::getStretchFactor)
        .def("clearQuery", &og::SPARS::clearQuery)
        .def("milestoneCount", &og::SPARS::milestoneCount)
        .def("guardCount", &og::SPARS::guardCount)
        .def("getIterationCount", &og::SPARS::getIterationCount)
        .def("getBestCost", &og::SPARS::getBestCost);
}
