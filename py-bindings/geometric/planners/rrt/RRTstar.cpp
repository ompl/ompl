#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRTstar(nb::module_ &m)
{
    nb::class_<og::RRTstar, ob::Planner>(m, "RRTstar")
        // Constructor
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        // solve
        .def("solve",
             [](og::RRTstar &self, nb::object what) {
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
             [](const og::RRTstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        // clear / setup
        .def("clear", &og::RRTstar::clear)
        .def("setup", &og::RRTstar::setup)

        // Goal bias
        .def("setGoalBias", &og::RRTstar::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::RRTstar::getGoalBias)

        // Range
        .def("setRange", &og::RRTstar::setRange, nb::arg("distance"))
        .def("getRange", &og::RRTstar::getRange)

        // Rewire factor
        .def("setRewireFactor", &og::RRTstar::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &og::RRTstar::getRewireFactor)

        // Delay collision checking
        .def("setDelayCC", &og::RRTstar::setDelayCC, nb::arg("delayCC"))
        .def("getDelayCC", &og::RRTstar::getDelayCC)

        // Tree pruning
        .def("setTreePruning", &og::RRTstar::setTreePruning, nb::arg("prune"))
        .def("getTreePruning", &og::RRTstar::getTreePruning)

        // Prune threshold
        .def("setPruneThreshold", &og::RRTstar::setPruneThreshold, nb::arg("pp"))
        .def("getPruneThreshold", &og::RRTstar::getPruneThreshold)

        // Pruned measure
        .def("setPrunedMeasure", &og::RRTstar::setPrunedMeasure, nb::arg("informedMeasure"))
        .def("getPrunedMeasure", &og::RRTstar::getPrunedMeasure)

        // Informed sampling
        .def("setInformedSampling", &og::RRTstar::setInformedSampling, nb::arg("informedSampling"))
        .def("getInformedSampling", &og::RRTstar::getInformedSampling)

        // Sample rejection
        .def("setSampleRejection", &og::RRTstar::setSampleRejection, nb::arg("reject"))
        .def("getSampleRejection", &og::RRTstar::getSampleRejection)

        // New state rejection
        .def("setNewStateRejection", &og::RRTstar::setNewStateRejection, nb::arg("reject"))
        .def("getNewStateRejection", &og::RRTstar::getNewStateRejection)

        // Admissible cost to come
        .def("setAdmissibleCostToCome", &og::RRTstar::setAdmissibleCostToCome, nb::arg("admissible"))
        .def("getAdmissibleCostToCome", &og::RRTstar::getAdmissibleCostToCome)

        // Ordered sampling
        .def("setOrderedSampling", &og::RRTstar::setOrderedSampling, nb::arg("orderSamples"))
        .def("getOrderedSampling", &og::RRTstar::getOrderedSampling)

        // Batch size
        .def("setBatchSize", &og::RRTstar::setBatchSize, nb::arg("batchSize"))
        .def("getBatchSize", &og::RRTstar::getBatchSize)

        // Focus search
        .def("setFocusSearch", &og::RRTstar::setFocusSearch, nb::arg("focus"))
        .def("getFocusSearch", &og::RRTstar::getFocusSearch)

        // K-nearest
        .def("setKNearest", &og::RRTstar::setKNearest, nb::arg("useKNearest"))
        .def("getKNearest", &og::RRTstar::getKNearest)

        // Number of sampling attempts
        .def("setNumSamplingAttempts", &og::RRTstar::setNumSamplingAttempts, nb::arg("numAttempts"))
        .def("getNumSamplingAttempts", &og::RRTstar::getNumSamplingAttempts)

        // Statistics
        .def("numIterations", &og::RRTstar::numIterations)
        .def("bestCost", &og::RRTstar::bestCost);
}

