#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/fmt/BFMT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersFmt_BFMT(nb::module_ &m)
{
    nb::class_<og::BFMT, ob::Planner>(m, "BFMT")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::BFMT &self, nb::object what) {
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
             [](const og::BFMT &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        // clear / setup
        .def("clear", &og::BFMT::clear)
        .def("setup", &og::BFMT::setup)

        // Number of samples
        .def("setNumSamples", &og::BFMT::setNumSamples, nb::arg("numSamples"))
        .def("getNumSamples", &og::BFMT::getNumSamples)

        // Nearest K
        .def("setNearestK", &og::BFMT::setNearestK, nb::arg("nearestK"))
        .def("getNearestK", &og::BFMT::getNearestK)

        // Radius multiplier
        .def("setRadiusMultiplier", &og::BFMT::setRadiusMultiplier, nb::arg("radiusMultiplier"))
        .def("getRadiusMultiplier", &og::BFMT::getRadiusMultiplier)

        // Free space volume
        .def("setFreeSpaceVolume", &og::BFMT::setFreeSpaceVolume, nb::arg("freeSpaceVolume"))
        .def("getFreeSpaceVolume", &og::BFMT::getFreeSpaceVolume)

        // Cache collision checking
        .def("setCacheCC", &og::BFMT::setCacheCC, nb::arg("ccc"))
        .def("getCacheCC", &og::BFMT::getCacheCC)

        // Heuristics
        .def("setHeuristics", &og::BFMT::setHeuristics, nb::arg("h"))
        .def("getHeuristics", &og::BFMT::getHeuristics)

        // Extended FMT
        .def("setExtendedFMT", &og::BFMT::setExtendedFMT, nb::arg("e"))
        .def("getExtendedFMT", &og::BFMT::getExtendedFMT)

        // BFMT-specific settings
        .def("setFreeSpaceVolume", &og::BFMT::setFreeSpaceVolume, nb::arg("freeSpaceVolume"))
        .def("getFreeSpaceVolume", &og::BFMT::getFreeSpaceVolume);
}

