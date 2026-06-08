#include <limits>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalLazySamples(nb::module_ &m)
{
    nb::class_<ob::GoalLazySamples, ob::GoalStates>(m, "GoalLazySamples")
        .def(nb::init<const ob::SpaceInformationPtr &, ob::GoalSamplingFn, bool, double>())
        .def("sampleGoal", &ob::GoalLazySamples::sampleGoal, nb::arg("state"))
        .def("distanceGoal", &ob::GoalLazySamples::distanceGoal, nb::arg("state"))
        .def("addState", &ob::GoalLazySamples::addState, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalLazySamples::maxSampleCount)
        .def("startSampling", &ob::GoalLazySamples::startSampling)
        .def("stopSampling", &ob::GoalLazySamples::stopSampling)
        .def("isSampling", &ob::GoalLazySamples::isSampling)
        .def("samplingAttemptsCount", &ob::GoalLazySamples::samplingAttemptsCount,
             "Total calls to the sampler function so far.")
        .def("setMinNewSampleDistance", &ob::GoalLazySamples::setMinNewSampleDistance, nb::arg("dist"),
             "Require new samples to be at least this far from all existing ones.")
        .def("getMinNewSampleDistance", &ob::GoalLazySamples::getMinNewSampleDistance)
        .def("setNewStateCallback", &ob::GoalLazySamples::setNewStateCallback, nb::arg("callback"))
        .def("addStateIfDifferent", &ob::GoalLazySamples::addStateIfDifferent, nb::arg("state"), nb::arg("minDistance"))
        .def("couldSample", &ob::GoalLazySamples::couldSample)
        .def("hasStates", &ob::GoalLazySamples::hasStates)
        .def("getState", &ob::GoalLazySamples::getState, nb::arg("index"), nb::rv_policy::reference_internal)
        .def("getStateCount", &ob::GoalLazySamples::getStateCount)
        .def("clear", &ob::GoalLazySamples::clear);
}
