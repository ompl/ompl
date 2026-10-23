#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARStwo(nb::module_ &m)
{
    nb::class_<og::SPARStwo, ob::Planner>(m, "SPARStwo")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "constructRoadmap",
            [](og::SPARStwo &self, nb::handle what, bool stopOnMaxFail)
            {
                const ob::PlannerTerminationCondition ptc = ompl::binding::toTerminationCondition(what);
                nb::gil_scoped_release release;
                self.constructRoadmap(ptc, stopOnMaxFail);
            },
            nb::arg("ptc"), nb::arg("stopOnMaxFail") = false)
        .def("solve", ompl::binding::solveBinding<og::SPARStwo>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::SPARStwo>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::SPARStwo &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SPARStwo::clear)
        .def("setup", &og::SPARStwo::setup)
        .def("setStretchFactor", &og::SPARStwo::setStretchFactor, nb::arg("t"))
        .def("setSparseDeltaFraction", &og::SPARStwo::setSparseDeltaFraction, nb::arg("D"))
        .def("setDenseDeltaFraction", &og::SPARStwo::setDenseDeltaFraction, nb::arg("d"))
        .def("setMaxFailures", &og::SPARStwo::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARStwo::getMaxFailures)
        .def("getDenseDeltaFraction", &og::SPARStwo::getDenseDeltaFraction)
        .def("getSparseDeltaFraction", &og::SPARStwo::getSparseDeltaFraction)
        .def("getStretchFactor", &og::SPARStwo::getStretchFactor)
        .def("clearQuery", &og::SPARStwo::clearQuery)
        .def("milestoneCount", &og::SPARStwo::milestoneCount)
        .def("getIterationCount", &og::SPARStwo::getIterationCount)
        .def("getBestCost", &og::SPARStwo::getBestCost);
}
