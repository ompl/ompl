#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
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
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace oc = ompl::control;

// Helper dictionary access
static PyObject **get_dict_ptr(PyObject *obj)
{
    return _PyObject_GetDictPtr(obj);
}

int control_space_information_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;

    // 1. Visit __dict__
    PyObject **dictptr = get_dict_ptr(self);
    if (dictptr && *dictptr)
    {
        Py_VISIT(*dictptr);
    }

    try
    {
        auto *si = nb::inst_ptr<oc::SpaceInformation>(self);
        if (si)
        {
            // Visit Propagator
            auto prop = si->getStatePropagator();
            if (prop)
            {
                nb::handle h = nb::find(prop);
                if (h.is_valid())
                {
                    Py_VISIT(h.ptr());
                }
                else if (dictptr && *dictptr)
                {
                    PyObject *item = PyDict_GetItemString(*dictptr, "_prop");
                    if (item)
                        Py_VISIT(item);
                }
            }
            // Visit SVC (Base)
            auto svc = si->getStateValidityChecker();
            if (svc)
            {
                nb::handle h = nb::find(svc);
                if (h.is_valid())
                {
                    Py_VISIT(h.ptr());
                }
                else if (dictptr && *dictptr)
                {
                    PyObject *item = PyDict_GetItemString(*dictptr, "_svc");
                    if (item)
                        Py_VISIT(item);
                }
            }
        }
    }
    catch (...)
    {
    }
    return 0;
}

int control_space_information_tp_clear(PyObject *self)
{
    // 1. Clear __dict__
    PyObject **dictptr = get_dict_ptr(self);
    if (dictptr && *dictptr)
    {
        Py_CLEAR(*dictptr);
    }
    // 2. Break C++ Cycle
    try
    {
        auto *si = nb::inst_ptr<oc::SpaceInformation>(self);
        if (si)
        {
            si->setStatePropagator(oc::StatePropagatorPtr(nullptr));
            si->setStateValidityChecker(ob::StateValidityCheckerPtr(nullptr));
        }
    }
    catch (...)
    {
    }
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

        // allocControl, freeControl, copyControl, cloneControl
        .def("allocControl", &oc::SpaceInformation::allocControl)
        .def("freeControl", &oc::SpaceInformation::freeControl, nb::arg("control"))
        .def("copyControl", &oc::SpaceInformation::copyControl, nb::arg("destination"), nb::arg("source"))
        .def("cloneControl", &oc::SpaceInformation::cloneControl, nb::arg("source"))

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
            "setStatePropagator",
            [](oc::SpaceInformation &si, const ompl::control::StatePropagatorFn &sp)
            {
                si.setStatePropagator(sp);
                nb::object self = nb::find(nb::cast(&si));
                if (self.is_valid())
                    nb::setattr(self, "_prop", nb::cast(sp));
            },
            nb::arg("sp"))
        .def(
            "setStatePropagator",
            [](oc::SpaceInformation &si, const oc::StatePropagatorPtr &sp)
            {
                si.setStatePropagator(sp);
                nb::object self = nb::find(nb::cast(&si));
                if (self.is_valid())
                    nb::setattr(self, "_prop", nb::cast(sp));
            },
            nb::arg("sp"))

        // setPropagationStepSize, getPropagationStepSize
        .def("setPropagationStepSize", &oc::SpaceInformation::setPropagationStepSize, nb::arg("stepSize"))
        .def("getPropagationStepSize", &oc::SpaceInformation::getPropagationStepSize)

        // propagate(...) overloads
        .def(
            "propagate",
            [](const oc::SpaceInformation &si, const ob::State *state, const oc::Control *control, int steps,
               ob::State *result) { si.propagate(state, control, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        .def("canPropagateBackward", &oc::SpaceInformation::canPropagateBackward)
        .def(
            "propagateWhileValid",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps,
               ob::State *result) { return si.propagateWhileValid(start, ctrl, steps, result); },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("result"))
        // overload that writes to a vector<State*>...
        .def(
            "propagateWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps, bool alloc)
            {
                // We'll wrap the version that writes to a std::vector<base::State*>.
                std::vector<ob::State *> path;
                si.propagate(start, ctrl, steps, path, alloc);
                return path;  // returns a Python list of pointers
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("alloc") = true)
        .def(
            "propagateWhileValidWithAlloc",
            [](const oc::SpaceInformation &si, const ob::State *start, const oc::Control *ctrl, int steps, bool alloc)
            {
                std::vector<ob::State *> path;
                unsigned int nValid = si.propagateWhileValid(start, ctrl, steps, path, alloc);
                return std::make_pair(nValid, path);
            },
            nb::arg("start"), nb::arg("control"), nb::arg("steps"), nb::arg("alloc") = true)

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
