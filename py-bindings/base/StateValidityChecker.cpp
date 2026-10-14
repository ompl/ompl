#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/tuple.h>
#include <thread>
#include <tuple>
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_StateValidityChecker(nb::module_ &m)
{
    // Trampoline for StateValidityChecker to allow Python subclassing
    struct PyStateValidityChecker : ob::StateValidityChecker
    {
        NB_TRAMPOLINE(ob::StateValidityChecker, 5);

        // Pure virtual: must be overridden in Python
        bool isValid(const ob::State *state) const override
        {
            if (std::this_thread::get_id() == owner_)
            {
                NB_OVERRIDE_PURE(isValid, state);
            }
            // An exception escaping a planner's or GoalLazySamples' worker thread would std::terminate.
            nb::gil_scoped_acquire gil;
            try
            {
                NB_OVERRIDE_PURE(isValid, state);
            }
            catch (nb::python_error &e)
            {
                e.discard_as_unraisable(nb_trampoline.base());
            }
            catch (const nb::cast_error &)
            {
                PyErr_SetString(PyExc_TypeError, "a state validity checker must return a bool");
                PyErr_WriteUnraisable(nb_trampoline.base().ptr());
            }
            return false;
        }

        // Out-parameter overloads dispatch to their own Python names, which return what C++ writes back.
        bool isValid(const ob::State *state, double &dist) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "isValidWithClearance", false);
            if (!nb_ticket.key.is_valid())
                return ob::StateValidityChecker::isValid(state, dist);
            auto [valid, clearance] =
                nb::cast<std::pair<bool, double>>(nb_trampoline.base().attr(nb_ticket.key)(state));
            dist = clearance;
            return valid;
        }

        bool isValid(const ob::State *state, double &dist, ob::State *validState,
                     bool &validStateAvailable) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "isValidWithValidState", false);
            if (!nb_ticket.key.is_valid())
                return ob::StateValidityChecker::isValid(state, dist, validState, validStateAvailable);
            auto [valid, clearance, available] =
                nb::cast<std::tuple<bool, double, bool>>(nb_trampoline.base().attr(nb_ticket.key)(state, validState));
            dist = clearance;
            validStateAvailable = available;
            return valid;
        }

        double clearance(const ob::State *state) const override
        {
            NB_OVERRIDE(clearance, state);
        }

        double clearance(const ob::State *state, ob::State *validState, bool &validStateAvailable) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "clearanceWithValidState", false);
            if (!nb_ticket.key.is_valid())
                return ob::StateValidityChecker::clearance(state, validState, validStateAvailable);
            auto [dist, available] =
                nb::cast<std::pair<double, bool>>(nb_trampoline.base().attr(nb_ticket.key)(state, validState));
            validStateAvailable = available;
            return dist;
        }

    private:
        std::thread::id owner_{std::this_thread::get_id()};
    };

    nb::class_<ob::StateValidityChecker, PyStateValidityChecker>(m, "StateValidityChecker")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("isValid",
             static_cast<bool (ob::StateValidityChecker::*)(const ob::State *) const>(
                 &ob::StateValidityChecker::isValid),
             nb::arg("state"))
        .def(
            "isValidWithClearance",
            [](const ob::StateValidityChecker &svc, const ob::State *state)
            {
                double dist = 0.0;
                bool valid = svc.isValid(state, dist);
                return std::make_pair(valid, dist);
            },
            nb::arg("state"))
        .def(
            "isValidWithValidState",
            [](const ob::StateValidityChecker &svc, const ob::State *state, ob::State *validState)
            {
                double dist = 0.0;
                bool available = false;
                bool valid = svc.isValid(state, dist, validState, available);
                return std::make_tuple(valid, dist, available);
            },
            nb::arg("state"), nb::arg("validState"))
        .def("clearance",
             static_cast<double (ob::StateValidityChecker::*)(const ob::State *) const>(
                 &ob::StateValidityChecker::clearance),
             nb::arg("state"))
        .def(
            "clearanceWithValidState",
            [](const ob::StateValidityChecker &svc, const ob::State *state, ob::State *validState)
            {
                bool available = false;
                double dist = svc.clearance(state, validState, available);
                return std::make_pair(dist, available);
            },
            nb::arg("state"), nb::arg("validState"))
        .def("getSpecs", &ob::StateValidityChecker::getSpecs);

    nb::class_<ob::AllValidStateValidityChecker, ob::StateValidityChecker>(m, "AllValidStateValidityChecker")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));

    // Bind StateValidityCheckerSpecs
    nb::class_<ob::StateValidityCheckerSpecs>(m, "StateValidityCheckerSpecs")
        .def(nb::init<>())
        .def_rw("clearanceComputationType", &ob::StateValidityCheckerSpecs::clearanceComputationType)
        .def_rw("hasValidDirectionComputation", &ob::StateValidityCheckerSpecs::hasValidDirectionComputation);

    // Bind ClearanceComputationType enum
    nb::enum_<ob::StateValidityCheckerSpecs::ClearanceComputationType>(m, "ClearanceComputationType")
        .value("NONE", ob::StateValidityCheckerSpecs::NONE)
        .value("EXACT", ob::StateValidityCheckerSpecs::EXACT)
        .value("APPROXIMATE", ob::StateValidityCheckerSpecs::APPROXIMATE)
        .value("BOUNDED_APPROXIMATE", ob::StateValidityCheckerSpecs::BOUNDED_APPROXIMATE)
        .export_values();
}
