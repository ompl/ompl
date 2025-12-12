#include <limits>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalLazySamples(nb::module_ &m)
{
     // TODO [ob::GoalLazySamples][TEST]
    nb::class_<ob::GoalLazySamples, ob::GoalStates>(m, "GoalLazySamples")
        // custom constructor: wrap a Python callable into the C++ GoalSamplingFn
        .def(nb::init<const ob::SpaceInformationPtr &, ob::GoalSamplingFn, bool, double>())

        // core overrides from GoalStates / GoalSampleableRegion:
        .def("sampleGoal", &ob::GoalLazySamples::sampleGoal, nb::arg("state"),
             "Generate a new goal sample into `state` (blocking if necessary).")
        .def("distanceGoal", &ob::GoalLazySamples::distanceGoal, nb::arg("state"),
             "Return distance from `state` to the nearest sampled goal.")

        // GoalStates overrides:
        .def("addState", &ob::GoalLazySamples::addState, nb::arg("state"), "Add a fixed goal state.")
        .def("maxSampleCount", &ob::GoalLazySamples::maxSampleCount,
             "Maximum number of goal samples (unbounded → huge value).")

        // lazy‐sampling–specific controls:
        .def("startSampling", &ob::GoalLazySamples::startSampling,
             "Begin background sampling thread (ignored if already running).")
        .def("stopSampling", &ob::GoalLazySamples::stopSampling,
             "Signal the background thread to stop (and block until it does).")
        .def("isSampling", &ob::GoalLazySamples::isSampling, "Return true if the background sampling thread is alive.")
        .def("samplingAttemptsCount", &ob::GoalLazySamples::samplingAttemptsCount,
             "Total calls to the sampler function so far.")
        .def("setMinNewSampleDistance", &ob::GoalLazySamples::setMinNewSampleDistance, nb::arg("dist"),
             "Require new samples to be at least this far from all existing ones.")
        .def("getMinNewSampleDistance", &ob::GoalLazySamples::getMinNewSampleDistance,
             "Get the current minimum-distance threshold.")

        // callback when a new sample is accepted
        .def("setNewStateCallback", &ob::GoalLazySamples::setNewStateCallback, nb::arg("callback"))

        // lower‐level control:
        .def("addStateIfDifferent", &ob::GoalLazySamples::addStateIfDifferent, nb::arg("state"), nb::arg("minDistance"),
             "Add `state` only if it is ≥ `minDistance` from all existing samples;\n"
             "returns True if added.")

        // saturating overrides:
        .def("couldSample", &ob::GoalLazySamples::couldSample,
             "True if samplerFn reports it could generate more (i.e. not exhausted).")
        .def("hasStates", &ob::GoalLazySamples::hasStates, "True if any goal states (fixed or sampled) exist.")
        .def("getState", &ob::GoalLazySamples::getState, nb::arg("index"), nb::rv_policy::reference_internal,
             "Get the goal state at index (0 ≤ index < getStateCount()).")
        .def("getStateCount", &ob::GoalLazySamples::getStateCount, "Number of goal states accumulated so far.")

        // inherited clear
        .def("clear", &ob::GoalLazySamples::clear, "Remove all fixed and sampled goal states.");
}
