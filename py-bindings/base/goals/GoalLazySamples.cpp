#include <atomic>
#include <limits>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "PyGC.h"
#include "../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;

namespace
{
    /// Holds the Python callbacks in members rather than inside the std::functions GoalLazySamples stores.
    /// A callable handed to those through nanobind's caster stays alive on a bare Py_INCREF the collector
    /// cannot see, so a sampler that reaches back to its own goal -- a bound method, a closure over the
    /// SpaceInformation -- could never be collected.
    struct PyGoalLazySamples : ob::GoalLazySamples
    {
        nb::object sampler;
        nb::object newStateCallback;
        nb::handle wrapper;
        std::atomic<bool> dying{false};

        PyGoalLazySamples(const ob::SpaceInformationPtr &si, nb::object fn, bool autoStart, double minDist)
          : ob::GoalLazySamples(
                si,
                [this](const ob::GoalLazySamples *, ob::State *state)
                {
                    nb::gil_scoped_acquire gil;
                    // Cleared by tp_clear, or the destructor is waiting on this thread. Either way, report
                    // exhaustion and let the sampling thread wind down.
                    if (dying.load(std::memory_order_relaxed) || !sampler.is_valid())
                        return false;
                    return nb::cast<bool>(sampler(wrapper, state));
                },
                // Deferred: the base constructor starts the sampling thread, which would call the sampler
                // before the member holding it is initialised.
                false, minDist)
          , sampler(std::move(fn))
        {
            // Borrowed on purpose. This object lives inside its Python wrapper, so the wrapper outlives it and
            // an owning reference would be a cycle nothing could break -- and a reference taken per sampling
            // pass would hand the last one to the sampling thread, moving destruction (and the join below)
            // onto the very thread being joined.
            wrapper = nb::find(*this);
            if (!wrapper.is_valid())
                wrapper = nb::handle(Py_None);
            if (autoStart)
                startSampling();
        }

        ~PyGoalLazySamples() override
        {
            // ~GoalLazySamples() joins the sampling thread too, but only once these members are gone, so the
            // wait has to happen here while they are still alive. `dying` stops the thread from reaching for
            // the Python wrapper, which is already being deallocated; the GIL has to be released because the
            // thread needs it to finish the pass it is on before it can see the termination flag.
            dying.store(true, std::memory_order_relaxed);
            nb::gil_scoped_release release;
            stopSampling();
        }

        // The stored callbacks capture `this`.
        PyGoalLazySamples(const PyGoalLazySamples &) = delete;
        PyGoalLazySamples &operator=(const PyGoalLazySamples &) = delete;
    };
}  // namespace

void ompl::binding::base::initGoals_GoalLazySamples(nb::module_ &m)
{
    nb::class_<PyGoalLazySamples, ob::GoalStates>(
        m, "GoalLazySamples",
        nb::type_slots(
            gc::gcSlots<PyGoalLazySamples, &PyGoalLazySamples::sampler, &PyGoalLazySamples::newStateCallback>))
        .def(
            "__init__",
            [](PyGoalLazySamples *self, const ob::SpaceInformationPtr &si, nb::callable samplerFunc, bool autoStart,
               double minDist) { new (self) PyGoalLazySamples(si, std::move(samplerFunc), autoStart, minDist); },
            nb::arg("si"), nb::arg("samplerFunc"), nb::arg("autoStart") = true,
            nb::arg("minDist") = std::numeric_limits<double>::epsilon())
        .def("sampleGoal", &ob::GoalLazySamples::sampleGoal, nb::arg("state"))
        .def("distanceGoal", &ob::GoalLazySamples::distanceGoal, nb::arg("state"))
        .def("addState", &ob::GoalLazySamples::addState, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalLazySamples::maxSampleCount)
        .def("startSampling", &ob::GoalLazySamples::startSampling)
        // Joining the sampling thread while holding the GIL would deadlock: the thread needs it to call the
        // sampler one last time before it can see the termination flag.
        .def("stopSampling", &ob::GoalLazySamples::stopSampling, nb::call_guard<nb::gil_scoped_release>())
        .def("isSampling", &ob::GoalLazySamples::isSampling)
        .def("samplingAttemptsCount", &ob::GoalLazySamples::samplingAttemptsCount,
             "Total calls to the sampler function so far.")
        .def("setMinNewSampleDistance", &ob::GoalLazySamples::setMinNewSampleDistance, nb::arg("dist"),
             "Require new samples to be at least this far from all existing ones.")
        .def("getMinNewSampleDistance", &ob::GoalLazySamples::getMinNewSampleDistance)
        .def(
            "setNewStateCallback",
            [](PyGoalLazySamples &gls, nb::callable callback)
            {
                gls.newStateCallback = std::move(callback);
                gls.setNewStateCallback(
                    [&gls](const ob::State *state)
                    {
                        nb::gil_scoped_acquire gil;
                        if (gls.newStateCallback.is_valid())
                            gls.newStateCallback(state);
                    });
            },
            nb::arg("callback"))
        .def("addStateIfDifferent", &ob::GoalLazySamples::addStateIfDifferent, nb::arg("state"), nb::arg("minDistance"))
        .def("couldSample", &ob::GoalLazySamples::couldSample)
        .def("hasStates", &ob::GoalLazySamples::hasStates)
        .def("getState", &ob::GoalLazySamples::getState, nb::arg("index"), nb::rv_policy::reference_internal)
        .def("getStateCount", &ob::GoalLazySamples::getStateCount)
        .def("clear", &ob::GoalLazySamples::clear);
}
