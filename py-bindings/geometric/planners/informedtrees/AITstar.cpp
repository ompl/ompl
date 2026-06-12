#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/AITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_AITstar(nb::module_ &m)
{
    nb::class_<og::AITstar, ob::Planner>(m, "AITstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "solve",
            [](og::AITstar &self, nb::object what)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                    return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                if (nb::isinstance<double>(what))
                    return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                throw nb::type_error(
                    "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
            })
        .def(
            "getPlannerData", [](const og::AITstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::AITstar::clear)
        .def("setup", &og::AITstar::setup)
        .def("bestCost", &og::AITstar::bestCost)
        .def("setBatchSize", &og::AITstar::setBatchSize, nb::arg("batchSize"))
        .def("getBatchSize", &og::AITstar::getBatchSize)
        .def("setRewireFactor", &og::AITstar::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &og::AITstar::getRewireFactor)
        .def("trackApproximateSolutions", &og::AITstar::trackApproximateSolutions, nb::arg("track"))
        .def("areApproximateSolutionsTracked", &og::AITstar::areApproximateSolutionsTracked)
        .def("enablePruning", &og::AITstar::enablePruning, nb::arg("prune"))
        .def("isPruningEnabled", &og::AITstar::isPruningEnabled)
        .def("setUseKNearest", &og::AITstar::setUseKNearest, nb::arg("useKNearest"))
        .def("getUseKNearest", &og::AITstar::getUseKNearest)
        .def("setMaxNumberOfGoals", &og::AITstar::setMaxNumberOfGoals, nb::arg("numberOfGoals"))
        .def("getMaxNumberOfGoals", &og::AITstar::getMaxNumberOfGoals);
}
