#include <atomic>
#include <limits>
#include <memory>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "ompl/base/SpaceInformation.h"
#include "PyGC.h"
#include "../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;

namespace
{
    int decrefPending(void *obj)
    {
        Py_DECREF(static_cast<PyObject *>(obj));
        return 0;
    }

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
        // Unreported pin held while sampling, so the destructor (which joins this thread) never runs on it.
        nb::object passRef;

        PyGoalLazySamples(const ob::SpaceInformationPtr &si, nb::object fn, bool autoStart, double minDist)
          : ob::GoalLazySamples(
                si, [this](const ob::GoalLazySamples *, ob::State *state) { return sample(state); },
                // Deferred: the base constructor starts the sampling thread, which would call the sampler
                // before the member holding it is initialised.
                false, minDist)
          , sampler(std::move(fn))
        {
            // Borrowed: this object lives inside its Python wrapper, so the wrapper outlives it.
            wrapper = nb::find(*this);
            if (!wrapper.is_valid())
                wrapper = nb::handle(Py_None);
            // Installed once, before the thread starts: OMPL reads it from the sampling thread without a lock.
            setNewStateCallback([this](const ob::State *state) { notify(state); });
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

        bool sample(ob::State *state)
        {
            nb::gil_scoped_acquire gil;
            if (dying.load(std::memory_order_relaxed) || !sampler.is_valid())
            {
                releasePass();
                return false;
            }
            if (!passRef.is_valid())
                passRef = nb::borrow(wrapper);
            else if (Py_REFCNT(passRef.ptr()) == 1)
            {
                // Nothing but this thread wants the goal any more.
                releasePass();
                return false;
            }

            nb::object fn = sampler;
            bool more = false;
            try
            {
                more = nb::cast<bool>(fn(wrapper, state));
            }
            catch (nb::python_error &e)
            {
                e.discard_as_unraisable(fn);
            }
            catch (const nb::cast_error &)
            {
                PyErr_SetString(PyExc_TypeError, "the GoalLazySamples sampler must return a bool");
                PyErr_WriteUnraisable(fn.ptr());
            }
            if (!more)
                releasePass();
            return more;
        }

        void notify(const ob::State *state)
        {
            nb::gil_scoped_acquire gil;
            if (dying.load(std::memory_order_relaxed) || !newStateCallback.is_valid())
                return;
            nb::object fn = newStateCallback;
            try
            {
                fn(state);
            }
            catch (nb::python_error &e)
            {
                e.discard_as_unraisable(fn);
            }
        }

        // A last reference is handed to the main thread rather than dropped here.
        void releasePass()
        {
            if (!passRef.is_valid())
                return;
            if (Py_REFCNT(passRef.ptr()) > 1)
            {
                passRef.reset();
                return;
            }
            // If the pending-call queue is full the goal leaks, which beats destroying it on this thread.
            (void)Py_AddPendingCall(decrefPending, passRef.release().ptr());
        }
    };

    int goalTraverse(PyObject *self, visitproc visit, void *arg)
    {
        if (int rc = gc::traverse<PyGoalLazySamples, &PyGoalLazySamples::sampler, &PyGoalLazySamples::newStateCallback>(
                self, visit, arg);
            rc != 0)
            return rc;
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return gc::visitHeld(nb::inst_ptr<PyGoalLazySamples>(self)->getSpaceInformation(), visit, arg);
    }

    PyType_Slot goalSlots[] = {
        {Py_tp_traverse, (void *)goalTraverse},
        {Py_tp_clear,
         (void *)gc::clear<PyGoalLazySamples, &PyGoalLazySamples::sampler, &PyGoalLazySamples::newStateCallback>},
        {0, 0}};
}  // namespace

void ompl::binding::base::initGoals_GoalLazySamples(nb::module_ &m)
{
    nb::class_<PyGoalLazySamples, ob::GoalStates>(m, "GoalLazySamples", nb::type_slots(goalSlots))
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
        .def("stopSampling",
             [](PyGoalLazySamples &gls)
             {
                 {
                     // The thread needs the GIL to finish its pass before it can see the termination flag.
                     nb::gil_scoped_release release;
                     gls.stopSampling();
                 }
                 gls.passRef.reset();
             })
        .def("isSampling", &ob::GoalLazySamples::isSampling)
        .def("samplingAttemptsCount", &ob::GoalLazySamples::samplingAttemptsCount,
             "Total calls to the sampler function so far.")
        .def("setMinNewSampleDistance", &ob::GoalLazySamples::setMinNewSampleDistance, nb::arg("dist"),
             "Require new samples to be at least this far from all existing ones.")
        .def("getMinNewSampleDistance", &ob::GoalLazySamples::getMinNewSampleDistance)
        .def(
            "setNewStateCallback", [](PyGoalLazySamples &gls, nb::callable callback)
            { gls.newStateCallback = std::move(callback); }, nb::arg("callback"))
        .def("addStateIfDifferent", &ob::GoalLazySamples::addStateIfDifferent, nb::arg("state"), nb::arg("minDistance"))
        .def("couldSample", &ob::GoalLazySamples::couldSample)
        .def("hasStates", &ob::GoalLazySamples::hasStates)
        .def("getStateCount", &ob::GoalLazySamples::getStateCount)
        .def("clear", &ob::GoalLazySamples::clear);
}
