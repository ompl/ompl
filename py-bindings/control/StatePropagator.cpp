#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>

#include "ompl/control/SpaceInformation.h"
#include "ompl/control/StatePropagator.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    struct PyStatePropagator : oc::StatePropagator
    {
        NB_TRAMPOLINE(oc::StatePropagator, 4);

        void propagate(const ob::State *state, const oc::Control *control, double duration,
                       ob::State *result) const override
        {
            NB_OVERRIDE_PURE(propagate, state, control, duration, result);
        }

        bool canPropagateBackward() const override
        {
            NB_OVERRIDE(canPropagateBackward);
        }

        double steer(const ob::State *from, const ob::State *to, oc::Control *result) const override
        {
            NB_OVERRIDE(steer, from, to, result);
        }

        bool canSteer() const override
        {
            NB_OVERRIDE(canSteer);
        }
    };
}  // namespace

void ompl::binding::control::init_StatePropagator(nb::module_ &m)
{
    // No keep_alive on si: si.setStatePropagator(MyPropagator(si)) would close a cycle the collector cannot see.
    nb::class_<oc::StatePropagator, PyStatePropagator>(m, "StatePropagator")
        .def(nb::init<oc::SpaceInformation *>(), nb::arg("si"))
        .def("propagate", &oc::StatePropagator::propagate, nb::arg("state"), nb::arg("control"), nb::arg("duration"),
             nb::arg("result"))
        .def("canPropagateBackward", &oc::StatePropagator::canPropagateBackward)
        .def("steer", &oc::StatePropagator::steer, nb::arg("source"), nb::arg("dest"), nb::arg("result"))
        .def("canSteer", &oc::StatePropagator::canSteer);
}
