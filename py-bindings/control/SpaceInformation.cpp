#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/control/SpaceInformation.h"
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/Control.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/base/SpaceInformation.h"  // ensure the base class is included
#include "PyGC.h"
#include "OwnedControl.h"
#include "init.h"
#include "base/init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    struct CallableStatePropagator : oc::StatePropagator, gc::HoldsCallable
    {
        CallableStatePropagator(oc::SpaceInformation *si, nb::object fn)
          : oc::StatePropagator(si), gc::HoldsCallable(std::move(fn))
        {
        }

        void propagate(const ob::State *state, const oc::Control *control, double duration,
                       ob::State *result) const override
        {
            gc::call<void>(callable, state, control, duration, result);
        }
    };

    // ob::State has a protected destructor, so each state handed to Python carries a deleter that frees it.
    std::vector<std::shared_ptr<ob::State>> ownedStates(const oc::SpaceInformation &si,
                                                        const std::vector<ob::State *> &states)
    {
        std::vector<std::shared_ptr<ob::State>> owned;
        owned.reserve(states.size());
        for (ob::State *state : states)
            owned.emplace_back(state, [space = si.getStateSpace()](ob::State *s) { space->freeState(s); });
        return owned;
    }

    template <typename Propagate>
    std::vector<std::shared_ptr<ob::State>> propagateAllocating(const oc::SpaceInformation &si, Propagate propagate)
    {
        std::vector<ob::State *> states;
        try
        {
            propagate(states);
        }
        catch (...)
        {
            for (ob::State *state : states)
                if (state)
                    si.freeState(state);
            throw;
        }
        return ownedStates(si, states);
    }
}  // namespace

int control_space_information_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;
    if (int rc = gc::visitDict(self, visit, arg))
        return rc;
    // A view of a C++-owned SpaceInformation does not own what the C++ object holds.
    if (!gc::pythonOwned(self))
        return 0;

    const auto &si = *nb::inst_ptr<oc::SpaceInformation>(self);
    if (int rc = ompl::binding::base::visitSpaceInformation(si, visit, arg))
        return rc;
    return gc::visitHeld(si.getStatePropagator(), visit, arg);
}

int control_space_information_tp_clear(PyObject *self)
{
    if (nb::inst_ready(self) && gc::pythonOwned(self))
    {
        auto &si = *nb::inst_ptr<oc::SpaceInformation>(self);
        ompl::binding::base::clearSpaceInformation(si);
        if (gc::holdsPython(si.getStatePropagator()))
            si.setStatePropagator(oc::StatePropagatorPtr());
    }
    gc::clearDict(self);
    return 0;
}

PyType_Slot control_space_information_slots[] = {{Py_tp_traverse, (void *)control_space_information_tp_traverse},
                                                 {Py_tp_clear, (void *)control_space_information_tp_clear},
                                                 {0, 0}};

void ompl::binding::control::init_SpaceInformation(nb::module_ &m)
{
    nb::class_<oc::SpaceInformation, ob::SpaceInformation>(
        m, "SpaceInformation", nb::type_slots(control_space_information_slots), nb::dynamic_attr())
        //
        // Constructor
        //
        .def(nb::init<ob::StateSpacePtr, oc::ControlSpacePtr>(), nb::arg("stateSpace"), nb::arg("controlSpace"))

        // getControlSpace
        .def("getControlSpace", &oc::SpaceInformation::getControlSpace)

        // allocControl, copyControl, cloneControl
        .def("allocControl", [](const oc::SpaceInformation &si)
             { return ompl::binding::control::ownedControl(si.getControlSpace(), si.allocControl()); })
        .def("copyControl", &oc::SpaceInformation::copyControl, nb::arg("destination"), nb::arg("source"))
        .def(
            "cloneControl", [](const oc::SpaceInformation &si, const oc::Control *source)
            { return ompl::binding::control::ownedControl(si.getControlSpace(), si.cloneControl(source)); },
            nb::arg("source"))

        // printControl
        .def(
            "printControl", [](const oc::SpaceInformation &si, const oc::Control *ctrl)
            { si.printControl(ctrl, std::cout); }, nb::arg("control"))

        // equalControls, nullControl
        .def("equalControls", &oc::SpaceInformation::equalControls, nb::arg("control1"), nb::arg("control2"))
        .def("nullControl", &oc::SpaceInformation::nullControl, nb::arg("control"))

        // allocControlSampler
        .def("allocControlSampler", &oc::SpaceInformation::allocControlSampler)

        // setMinMaxControlDuration, etc.
        .def("setMinMaxControlDuration", &oc::SpaceInformation::setMinMaxControlDuration, nb::arg("minSteps"),
             nb::arg("maxSteps"))
        .def("setMinControlDuration", &oc::SpaceInformation::setMinControlDuration, nb::arg("minSteps"))
        .def("setMaxControlDuration", &oc::SpaceInformation::setMaxControlDuration, nb::arg("maxSteps"))
        .def("getMinControlDuration", &oc::SpaceInformation::getMinControlDuration)
        .def("getMaxControlDuration", &oc::SpaceInformation::getMaxControlDuration)

        // Directed control sampler
        .def("allocDirectedControlSampler", &oc::SpaceInformation::allocDirectedControlSampler)
        .def("setDirectedControlSamplerAllocator", &oc::SpaceInformation::setDirectedControlSamplerAllocator,
             nb::arg("dcsa"))
        .def("clearDirectedSamplerAllocator", &oc::SpaceInformation::clearDirectedSamplerAllocator)

        // StatePropagator
        .def("getStatePropagator", &oc::SpaceInformation::getStatePropagator)

        // setStatePropagator
        .def(
            "setStatePropagator", [](oc::SpaceInformation &si, const oc::StatePropagatorPtr &sp)
            { si.setStatePropagator(sp); }, nb::arg("sp"))
        .def(
            "setStatePropagator", [](oc::SpaceInformation &si, nb::callable sp)
            { si.setStatePropagator(std::make_shared<CallableStatePropagator>(&si, std::move(sp))); }, nb::arg("sp"))

        // setPropagationStepSize, getPropagationStepSize
        .def("setPropagationStepSize", &oc::SpaceInformation::setPropagationStepSize, nb::arg("stepSize"))
        .def("getPropagationStepSize", &oc::SpaceInformation::getPropagationStepSize)

        // propagate(...) overloads
        .def(
            "propagate",
            [](const oc::SpaceInformation &si, const ob::State *state, const oc::Control *control, int steps,
               ob::State *result) { si.propagate(state, control, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        // The vector caster copies the list, but the pointers in it still name the caller's states.
        .def(
            "propagate",
            [](const oc::SpaceInformation &si, const ob::State *state, const oc::Control *control, int steps,
               std::vector<ob::State *> result) { si.propagate(state, control, steps, result, false); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        .def("canPropagateBackward", &oc::SpaceInformation::canPropagateBackward)
        .def(
            "propagateWhileValid",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps,
               ob::State *result) { return si.propagateWhileValid(start, ctrl, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        .def(
            "propagateWhileValid",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps,
               std::vector<ob::State *> result) { return si.propagateWhileValid(start, ctrl, steps, result, false); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        .def(
            "propagateWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps)
            {
                return propagateAllocating(si, [&](std::vector<ob::State *> &states)
                                           { si.propagate(start, ctrl, steps, states, true); });
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"))
        .def(
            "propagateWhileValidWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps)
            {
                unsigned int nValid = 0;
                auto owned =
                    propagateAllocating(si, [&](std::vector<ob::State *> &states)
                                        { nValid = si.propagateWhileValid(start, ctrl, steps, states, true); });
                return std::make_pair(nValid, std::move(owned));
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"))

        // printSettings override
        .def("printSettings", [](const oc::SpaceInformation &si) { si.printSettings(std::cout); })
        .def("__repr__",
             [](const oc::SpaceInformation &si)
             {
                 std::ostringstream oss;
                 si.printSettings(oss);
                 return oss.str();
             })
        // setup override
        .def("setup", &oc::SpaceInformation::setup);
}
