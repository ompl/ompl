#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/AITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_AITstar(nb::module_ &m)
{
    nb::class_<og::AITstar, ob::Planner>(m, "AITstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::AITstar>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::AITstar>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
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
