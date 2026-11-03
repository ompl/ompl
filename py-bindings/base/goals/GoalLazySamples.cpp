#include <limits>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    // GoalLazySamples with a Python sampler. The sampling thread takes the GIL to call it, so:
    // - the destructor must not join that thread while holding the GIL, and
    // - once destruction starts the sampler must not be called: it would receive a dying object.
    // dying_ is only read and written with the GIL held, which orders the two threads. That also covers
    // construction: the thread may start in the base constructor, but it needs the GIL, which is held
    // until this constructor (called from Python) returns.
    struct PyGoalLazySamples final : ob::GoalLazySamples
    {
        PyGoalLazySamples(const ob::SpaceInformationPtr &si, ob::GoalSamplingFn samplerFunc, bool autoStart,
                          double minDist)
          : ob::GoalLazySamples(si, guarded(std::move(samplerFunc)), autoStart, minDist)
        {
        }

        ~PyGoalLazySamples() override
        {
            nb::gil_scoped_acquire gil;
            dying_ = true;
            nb::gil_scoped_release release;
            stopSampling();
        }

    private:
        static ob::GoalSamplingFn guarded(ob::GoalSamplingFn samplerFunc)
        {
            return [samplerFunc = std::move(samplerFunc)](const ob::GoalLazySamples *gls, ob::State *state)
            {
                nb::gil_scoped_acquire gil;
                return !static_cast<const PyGoalLazySamples *>(gls)->dying_ && samplerFunc(gls, state);
            };
        }

        bool dying_{false};
    };
}  // namespace

void ompl::binding::base::initGoals_GoalLazySamples(nb::module_ &m)
{
    // Exposed to Python as GoalLazySamples; the methods below are ob::GoalLazySamples'.
    nb::class_<PyGoalLazySamples, ob::GoalStates>(m, "GoalLazySamples")
        .def(nb::init<const ob::SpaceInformationPtr &, ob::GoalSamplingFn, bool, double>(), nb::arg("si"),
             nb::arg("samplerFunc"), nb::arg("autoStart") = true,
             nb::arg("minDist") = std::numeric_limits<double>::epsilon())
        .def("sampleGoal", &ob::GoalLazySamples::sampleGoal, nb::arg("state"))
        .def("distanceGoal", &ob::GoalLazySamples::distanceGoal, nb::arg("state"))
        .def("addState", &ob::GoalLazySamples::addState, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalLazySamples::maxSampleCount)
        .def("startSampling", &ob::GoalLazySamples::startSampling)
        .def("stopSampling", &ob::GoalLazySamples::stopSampling, nb::call_guard<nb::gil_scoped_release>())
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
