#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/PRM.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

void ompl::binding::geometric::initPlannersPrm_PRM(nb::module_ &m)
{
    // TAG [og::PRM][Planner]
    nb::class_<PRM, Planner>(m, "PRM")
        .def(nb::init<const SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("starStrategy") = false)
        .def("setMaxNearestNeighbors", &PRM::setMaxNearestNeighbors, nb::arg("k"))
        .def("getMaxNearestNeighbors", &PRM::getMaxNearestNeighbors)
        .def("setDefaultConnectionStrategy", &PRM::setDefaultConnectionStrategy)
        .def("constructRoadmap", &PRM::constructRoadmap, nb::arg("ptc"), nb::call_guard<nb::gil_scoped_release>())
        .def(
            "constructRoadmap", [](PRM &self, double time)
            { self.constructRoadmap(timedPlannerTerminationCondition(time)); }, nb::arg("time"),
            nb::call_guard<nb::gil_scoped_release>())
        .def("growRoadmap", nb::overload_cast<double>(&PRM::growRoadmap), nb::arg("growTime"), nb::call_guard<nb::gil_scoped_release>())
        .def("growRoadmapPtc", nb::overload_cast<const PlannerTerminationCondition &>(&PRM::growRoadmap),
             nb::arg("ptc"), nb::call_guard<nb::gil_scoped_release>())
        .def("expandRoadmap", nb::overload_cast<double>(&PRM::expandRoadmap), nb::arg("expandTime"), nb::call_guard<nb::gil_scoped_release>())
        .def("expandRoadmapPtc", nb::overload_cast<const PlannerTerminationCondition &>(&PRM::expandRoadmap),
             nb::arg("ptc"), nb::call_guard<nb::gil_scoped_release>())
        .def("setup", &PRM::setup)
        .def("clear", &PRM::clear)
        .def("clearQuery", &PRM::clearQuery)
        .def("getPlannerData", &PRM::getPlannerData, nb::arg("data"))
        .def("milestoneCount", &PRM::milestoneCount)
        .def("edgeCount", &PRM::edgeCount);
}
