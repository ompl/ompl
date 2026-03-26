#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/PRM.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

void ompl::binding::geometric::initPlannersPrm_PRM(nb::module_& m)
{
    // TAG [og::PRM][Planner]
    nb::class_<PRM, Planner>(m, "PRM")
        .def(nb::init<const SpaceInformationPtr&, bool>(),
             nb::arg("si"),
             nb::arg("starStrategy") = false)
        .def("setMaxNearestNeighbors", &PRM::setMaxNearestNeighbors,
             nb::arg("k"))
        .def("getMaxNearestNeighbors", &PRM::getMaxNearestNeighbors)
        .def("setDefaultConnectionStrategy", &PRM::setDefaultConnectionStrategy)
        .def("constructRoadmap", 
               [](PRM &self, nb::object what) {
                 if (nb::isinstance<PlannerTerminationCondition>(what)) {
                     return self.constructRoadmap(nb::cast<PlannerTerminationCondition>(what));
                 } else if (nb::isinstance<double>(what)) {
                     return self.constructRoadmap(timedPlannerTerminationCondition(nb::cast<double>(what)));
                 } else {
                     throw nb::type_error("Invalid argument type for constructRoadmap. Expected PlannerTerminationCondition or double.");
                 }
               })
        .def("growRoadmap", 
             nb::overload_cast<double>(&PRM::growRoadmap),
             nb::arg("growTime"))
        .def("growRoadmapPtc", 
             nb::overload_cast<const PlannerTerminationCondition&>(&PRM::growRoadmap),
             nb::arg("ptc"))
        .def("expandRoadmap", 
             nb::overload_cast<double>(&PRM::expandRoadmap),
             nb::arg("expandTime"))
        .def("expandRoadmapPtc", 
             nb::overload_cast<const PlannerTerminationCondition&>(&PRM::expandRoadmap),
             nb::arg("ptc"))
        .def("setup", &PRM::setup)
        .def("clear", &PRM::clear)
        .def("clearQuery", &PRM::clearQuery)
        .def("solve", 
             [](PRM &self, nb::object what) {
                 if (nb::isinstance<PlannerTerminationCondition>(what)) {
                     return self.solve(nb::cast<PlannerTerminationCondition>(what));
                 } else if (nb::isinstance<double>(what)) {
                     return self.solve(timedPlannerTerminationCondition(nb::cast<double>(what)));
                 } else {
                     throw nb::type_error("Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def("getPlannerData", &PRM::getPlannerData,
             nb::arg("data"))
        .def("milestoneCount", &PRM::milestoneCount)
        .def("edgeCount", &PRM::edgeCount);
}

