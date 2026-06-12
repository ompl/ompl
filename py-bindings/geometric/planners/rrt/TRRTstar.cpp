#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/TRRTstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_TRRTstar(nb::module_ &m)
{
    nb::class_<og::TRRTstar, ob::Planner>(m, "TRRTstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::TRRTstar &self, nb::object what)
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
            "getPlannerData", [](const og::TRRTstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::TRRTstar::clear)
        .def("setup", &og::TRRTstar::setup)
        .def("setGoalBias", &og::TRRTstar::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::TRRTstar::getGoalBias)
        .def("setRange", &og::TRRTstar::setRange, nb::arg("distance"))
        .def("getRange", &og::TRRTstar::getRange)
        .def("setRewireFactor", &og::TRRTstar::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &og::TRRTstar::getRewireFactor)
        .def("setDelayCC", &og::TRRTstar::setDelayCC, nb::arg("delayCC"))
        .def("getDelayCC", &og::TRRTstar::getDelayCC)
        .def("setTreePruning", &og::TRRTstar::setTreePruning, nb::arg("prune"))
        .def("getTreePruning", &og::TRRTstar::getTreePruning)
        .def("setPruneThreshold", &og::TRRTstar::setPruneThreshold, nb::arg("pp"))
        .def("getPruneThreshold", &og::TRRTstar::getPruneThreshold)
        .def("setAdmissibleCostToCome", &og::TRRTstar::setAdmissibleCostToCome, nb::arg("admissible"))
        .def("getAdmissibleCostToCome", &og::TRRTstar::getAdmissibleCostToCome)
        .def("setBatchSize", &og::TRRTstar::setBatchSize, nb::arg("batchSize"))
        .def("getBatchSize", &og::TRRTstar::getBatchSize)
        .def("setKNearest", &og::TRRTstar::setKNearest, nb::arg("useKNearest"))
        .def("getKNearest", &og::TRRTstar::getKNearest)
        .def("setNumSamplingAttempts", &og::TRRTstar::setNumSamplingAttempts, nb::arg("numAttempts"))
        .def("getNumSamplingAttempts", &og::TRRTstar::getNumSamplingAttempts)
        .def("numIterations", &og::TRRTstar::numIterations)
        .def("bestCost", &og::TRRTstar::bestCost)
        .def("setTempChangeFactor", &og::TRRTstar::setTempChangeFactor, nb::arg("factor"))
        .def("getTempChangeFactor", &og::TRRTstar::getTempChangeFactor)
        .def("setCostThreshold", &og::TRRTstar::setCostThreshold, nb::arg("maxCost"))
        .def("getCostThreshold", &og::TRRTstar::getCostThreshold)
        .def("setInitTemperature", &og::TRRTstar::setInitTemperature, nb::arg("initTemperature"))
        .def("getInitTemperature", &og::TRRTstar::getInitTemperature);
}
