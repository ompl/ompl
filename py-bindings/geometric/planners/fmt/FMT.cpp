#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/fmt/FMT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersFmt_FMT(nb::module_ &m)
{
    nb::class_<og::FMT, ob::Planner>(m, "FMT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::FMT &self, nb::object what) {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what)) {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 } else if (nb::isinstance<double>(what)) {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 } else {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })

        // getPlannerData
        .def("getPlannerData",
             [](const og::FMT &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        // clear / setup
        .def("clear", &og::FMT::clear)
        .def("setup", &og::FMT::setup)

        // Number of samples
        .def("setNumSamples", &og::FMT::setNumSamples, nb::arg("numSamples"))
        .def("getNumSamples", &og::FMT::getNumSamples)

        // Nearest K
        .def("setNearestK", &og::FMT::setNearestK, nb::arg("nearestK"))
        .def("getNearestK", &og::FMT::getNearestK)

        // Radius multiplier
        .def("setRadiusMultiplier", &og::FMT::setRadiusMultiplier, nb::arg("radiusMultiplier"))
        .def("getRadiusMultiplier", &og::FMT::getRadiusMultiplier)

        // Free space volume
        .def("setFreeSpaceVolume", &og::FMT::setFreeSpaceVolume, nb::arg("freeSpaceVolume"))
        .def("getFreeSpaceVolume", &og::FMT::getFreeSpaceVolume)

        // Cache collision checking
        .def("setCacheCC", &og::FMT::setCacheCC, nb::arg("ccc"))
        .def("getCacheCC", &og::FMT::getCacheCC)

        // Heuristics
        .def("setHeuristics", &og::FMT::setHeuristics, nb::arg("h"))
        .def("getHeuristics", &og::FMT::getHeuristics)

        // Extended FMT
        .def("setExtendedFMT", &og::FMT::setExtendedFMT, nb::arg("e"))
        .def("getExtendedFMT", &og::FMT::getExtendedFMT);
}

