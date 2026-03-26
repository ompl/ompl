#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_BITstar(nb::module_ &m)
{
    nb::class_<og::BITstar, ob::Planner>(m, "BITstar")
        .def(nb::init<const ob::SpaceInformationPtr &, const std::string &>(),
             nb::arg("si"), nb::arg("name") = "kBITstar")

        // solve
        .def("solve",
             [](og::BITstar &self, nb::object what) {
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
             [](const og::BITstar &self, ob::PlannerData &data) { self.getPlannerData(data); },
             nb::arg("data"))

        // clear / setup
        .def("clear", &og::BITstar::clear)
        .def("setup", &og::BITstar::setup)

        // Statistics
        .def("numIterations", &og::BITstar::numIterations)
        .def("bestCost", &og::BITstar::bestCost)
        .def("numBatches", &og::BITstar::numBatches)

        // Settings
        .def("setRewireFactor", &og::BITstar::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &og::BITstar::getRewireFactor)

        .def("setSamplesPerBatch", &og::BITstar::setSamplesPerBatch, nb::arg("n"))
        .def("getSamplesPerBatch", &og::BITstar::getSamplesPerBatch)

        .def("setUseKNearest", &og::BITstar::setUseKNearest, nb::arg("useKNearest"))
        .def("getUseKNearest", &og::BITstar::getUseKNearest)

        .def("setStrictQueueOrdering", &og::BITstar::setStrictQueueOrdering, nb::arg("beStrict"))
        .def("getStrictQueueOrdering", &og::BITstar::getStrictQueueOrdering)

        .def("setPruning", &og::BITstar::setPruning, nb::arg("prune"))
        .def("getPruning", &og::BITstar::getPruning)

        .def("setPruneThresholdFraction", &og::BITstar::setPruneThresholdFraction, nb::arg("fractionalChange"))
        .def("getPruneThresholdFraction", &og::BITstar::getPruneThresholdFraction)

        .def("setDelayRewiringUntilInitialSolution", &og::BITstar::setDelayRewiringUntilInitialSolution,
             nb::arg("delayRewiring"))
        .def("getDelayRewiringUntilInitialSolution", &og::BITstar::getDelayRewiringUntilInitialSolution)

        .def("setJustInTimeSampling", &og::BITstar::setJustInTimeSampling, nb::arg("useJit"))
        .def("getJustInTimeSampling", &og::BITstar::getJustInTimeSampling)

        .def("setDropSamplesOnPrune", &og::BITstar::setDropSamplesOnPrune, nb::arg("dropSamples"))
        .def("getDropSamplesOnPrune", &og::BITstar::getDropSamplesOnPrune)

        .def("setStopOnSolnImprovement", &og::BITstar::setStopOnSolnImprovement, nb::arg("stopOnChange"))
        .def("getStopOnSolnImprovement", &og::BITstar::getStopOnSolnImprovement)

        .def("setConsiderApproximateSolutions", &og::BITstar::setConsiderApproximateSolutions,
             nb::arg("findApproximate"))
        .def("getConsiderApproximateSolutions", &og::BITstar::getConsiderApproximateSolutions);
}

