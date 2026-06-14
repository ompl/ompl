#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/STRRTstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_STRRTstar(nb::module_ &m)
{
    nb::class_<og::STRRTstar, ob::Planner>(m, "STRRTstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::STRRTstar &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def(
            "getPlannerData", [](const og::STRRTstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::STRRTstar::clear)
        .def("setup", &og::STRRTstar::setup)
        .def("setRange", &og::STRRTstar::setRange, nb::arg("distance"))
        .def("getRange", &og::STRRTstar::getRange)
        .def("getOptimumApproxFactor", &og::STRRTstar::getOptimumApproxFactor)
        .def("setOptimumApproxFactor", &og::STRRTstar::setOptimumApproxFactor, nb::arg("optimumApproxFactor"))
        .def("getRewiringState", &og::STRRTstar::getRewiringState)
        .def("setRewiringToOff", &og::STRRTstar::setRewiringToOff)
        .def("setRewiringToRadius", &og::STRRTstar::setRewiringToRadius)
        .def("setRewiringToKNearest", &og::STRRTstar::setRewiringToKNearest)
        .def("getRewireFactor", &og::STRRTstar::getRewireFactor)
        .def("setRewireFactor", &og::STRRTstar::setRewireFactor, nb::arg("v"))
        .def("getBatchSize", &og::STRRTstar::getBatchSize)
        .def("setBatchSize", &og::STRRTstar::setBatchSize, nb::arg("v"))
        .def("setTimeBoundFactorIncrease", &og::STRRTstar::setTimeBoundFactorIncrease, nb::arg("f"))
        .def("setInitialTimeBoundFactor", &og::STRRTstar::setInitialTimeBoundFactor, nb::arg("f"))
        .def("setSampleUniformForUnboundedTime", &og::STRRTstar::setSampleUniformForUnboundedTime, nb::arg("uniform"));
}
