#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/AORRTC.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_AORRTC(nb::module_ &m)
{
    nb::class_<og::AORRTC, ob::Planner>(m, "AORRTC")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve", ompl::binding::solveBinding<og::AORRTC>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::AORRTC>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)

        // getPlannerData
        .def(
            "getPlannerData", [](const og::AORRTC &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))

        // clear / setup
        .def("clear", &og::AORRTC::clear)
        .def("setup", &og::AORRTC::setup)

        // Range
        .def("setRange", &og::AORRTC::setRange, nb::arg("distance"))
        .def("getRange", &og::AORRTC::getRange)

        // Best cost
        .def("bestCost", &og::AORRTC::bestCost)

        .def("reset", &og::AORRTC::reset, nb::arg("solvedProblem"))
        .def("getPathSimplifier",
             static_cast<const og::PathSimplifierPtr &(og::AORRTC::*)() const>(&og::AORRTC::getPathSimplifier))
        .def("simplifySolution", &og::AORRTC::simplifySolution, nb::arg("p"), nb::arg("ptc"),
             nb::call_guard<nb::gil_scoped_release>());
}
