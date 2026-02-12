#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/shared_ptr.h>
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
            NB_OVERRIDE_PURE(isValid, state);
        }

        // Virtual with default implementation
        bool isValid(const ob::State *state, double &dist) const override
        {
            NB_OVERRIDE(isValid, state, dist);
        }

        // Virtual with default implementation
        bool isValid(const ob::State *state, double &dist, ob::State *validState,
                     bool &validStateAvailable) const override
        {
            NB_OVERRIDE(isValid, state, dist, validState, validStateAvailable);
        }

        // Virtual with default implementation
        double clearance(const ob::State *state) const override
        {
            NB_OVERRIDE(clearance, state);
        }

        // Virtual with default implementation
        double clearance(const ob::State *state, ob::State *validState, bool &validStateAvailable) const override
        {
            NB_OVERRIDE(clearance, state, validState, validStateAvailable);
        }
    };

    nb::class_<ob::StateValidityChecker, PyStateValidityChecker>(m, "StateValidityChecker")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("isValid",
             static_cast<bool (ob::StateValidityChecker::*)(const ob::State *) const>(
                 &ob::StateValidityChecker::isValid),
             nb::arg("state"))
        .def("clearance",
             static_cast<double (ob::StateValidityChecker::*)(const ob::State *) const>(
                 &ob::StateValidityChecker::clearance),
             nb::arg("state"))
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
