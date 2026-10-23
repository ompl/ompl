#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/lazyinformedtrees/BLITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersLazyinformedtrees_BLITstar(nb::module_ &m)
{
    nb::class_<og::BLITstar, ob::Planner>(m, "BLITstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::BLITstar>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::BLITstar>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::BLITstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::BLITstar::clear)
        .def("setup", &og::BLITstar::setup)
        .def("setBatchSize", &og::BLITstar::setBatchSize, nb::arg("batchSize"))
        .def("getBatchSize", &og::BLITstar::getBatchSize)
        .def("setRewireFactor", &og::BLITstar::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &og::BLITstar::getRewireFactor)
        .def("enablePruning", &og::BLITstar::enablePruning, nb::arg("prune"))
        .def("isPruningEnabled", &og::BLITstar::isPruningEnabled)
        .def("setUseKNearest", &og::BLITstar::setUseKNearest, nb::arg("useKNearest"))
        .def("getUseKNearest", &og::BLITstar::getUseKNearest)
        .def("setMaxNumberOfGoals", &og::BLITstar::setMaxNumberOfGoals, nb::arg("numberOfGoals"))
        .def("getMaxNumberOfGoals", &og::BLITstar::getMaxNumberOfGoals);
}
