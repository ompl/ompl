#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/EITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_EITstar(nb::module_ &m)
{
    nb::class_<og::EITstar, ob::Planner>(m, "EITstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "solve",
            [](og::EITstar &self, nb::object what)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                    return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                if (nb::isinstance<double>(what))
                    return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                throw nb::type_error(
                    "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
            })
        .def(
            "getPlannerData", [](const og::EITstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::EITstar::clear)
        .def("clearQuery", &og::EITstar::clearQuery)
        .def("setup", &og::EITstar::setup)
        .def("bestCost", &og::EITstar::bestCost)
        .def("setBatchSize", &og::EITstar::setBatchSize, nb::arg("numSamples"))
        .def("getBatchSize", &og::EITstar::getBatchSize)
        .def("setRadiusFactor", &og::EITstar::setRadiusFactor, nb::arg("factor"))
        .def("getRadiusFactor", &og::EITstar::getRadiusFactor)
        .def("setSuboptimalityFactor", &og::EITstar::setSuboptimalityFactor, nb::arg("factor"))
        .def("enablePruning", &og::EITstar::enablePruning, nb::arg("prune"))
        .def("isPruningEnabled", &og::EITstar::isPruningEnabled)
        .def("trackApproximateSolutions", &og::EITstar::trackApproximateSolutions, nb::arg("track"))
        .def("areApproximateSolutionsTracked", &og::EITstar::areApproximateSolutionsTracked)
        .def("setUseKNearest", &og::EITstar::setUseKNearest, nb::arg("useKNearest"))
        .def("getUseKNearest", &og::EITstar::getUseKNearest)
        .def("setMaxNumberOfGoals", &og::EITstar::setMaxNumberOfGoals, nb::arg("numberOfGoals"))
        .def("getMaxNumberOfGoals", &og::EITstar::getMaxNumberOfGoals);
}
