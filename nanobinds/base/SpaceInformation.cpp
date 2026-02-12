#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <sstream>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/ValidStateSampler.h"
#include "init.h"

namespace nb = nanobind;

static PyObject **get_dict_ptr(PyObject *obj)
{
    return _PyObject_GetDictPtr(obj);
}

int space_information_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;

    // 1. Visit __dict__ (handles references stored in Python attributes)
    PyObject **dictptr = get_dict_ptr(self);
    if (dictptr && *dictptr)
    {
        Py_VISIT(*dictptr);
    }

    try
    {
        auto *si = nb::inst_ptr<ompl::base::SpaceInformation>(self);
        if (si)
        {
            auto svc = si->getStateValidityChecker();
            if (svc)
            {
                // 2. Try to visit C++ child via nb::find (works for Python Classes)
                nb::handle h = nb::find(svc);
                if (h.is_valid())
                {
                    Py_VISIT(h.ptr());
                }
                else if (dictptr && *dictptr)
                {
                    // 3. If nb::find failed (Lambda), visit the proxy in __dict__ explicitely
                    // to account for the C++ reference (since Lambda + std::function = 2 refs).
                    PyObject *item = PyDict_GetItemString(*dictptr, "_svc");
                    if (item)
                    {
                        Py_VISIT(item);
                    }
                }
            }
        }
    }
    catch (...)
    {
    }
    return 0;
}

int space_information_tp_clear(PyObject *self)
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
        auto *si = nb::inst_ptr<ompl::base::SpaceInformation>(self);
        if (si)
        {
            si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(nullptr));
        }
    }
    catch (...)
    {
    }
    return 0;
}

PyType_Slot space_information_slots[] = {{Py_tp_traverse, (void *)space_information_tp_traverse},
                                         {Py_tp_clear, (void *)space_information_tp_clear},
                                         {0, 0}};

void ompl::binding::base::init_SpaceInformation(nb::module_ &m)
{
    nb::class_<ompl::base::SpaceInformation>(m, "SpaceInformation", nb::type_slots(space_information_slots), nb::dynamic_attr())
        .def(nb::init<ompl::base::StateSpacePtr>())
        .def("isValid", &ompl::base::SpaceInformation::isValid)
        .def("getStateSpace", &ompl::base::SpaceInformation::getStateSpace)
        .def("getStateDimension", &ompl::base::SpaceInformation::getStateDimension)
        .def("getSpaceMeasure", &ompl::base::SpaceInformation::getSpaceMeasure)
        .def("equalStates", &ompl::base::SpaceInformation::equalStates)
        .def("satisfiesBounds", &ompl::base::SpaceInformation::satisfiesBounds)
        .def("distance", &ompl::base::SpaceInformation::distance)
        .def("enforceBounds", &ompl::base::SpaceInformation::enforceBounds)
        .def("printState", [](const ompl::base::SpaceInformation &si, const ompl::base::State *state) { si.printState(state, std::cout); })
        .def("setStateValidityChecker", 
            [](ompl::base::SpaceInformation &si, const std::function<bool(const ompl::base::State*)> &func) {
                si.setStateValidityChecker(func);
                // Store in dict for traversal
                nb::object self = nb::find(nb::cast(&si)); // Should verify find works for self
                if (self.is_valid()) {
                    nb::setattr(self, "_svc", nb::cast(func));
                }
            },
            nb::arg("svc"))
        .def("setStateValidityChecker",
            [](ompl::base::SpaceInformation &si, const ompl::base::StateValidityCheckerPtr &svc) {
                si.setStateValidityChecker(svc);
                 // Store in dict for traversal
                nb::object self = nb::find(nb::cast(&si));
                if (self.is_valid()) {
                    nb::setattr(self, "_svc", nb::cast(svc));
                }
            },
            nb::arg("svc"))
        .def("getStateValidityChecker", &ompl::base::SpaceInformation::getStateValidityChecker)
        .def("setMotionValidator", &ompl::base::SpaceInformation::setMotionValidator)
        .def("getMotionValidator", nb::overload_cast<>(&ompl::base::SpaceInformation::getMotionValidator, nb::const_))
        .def("setStateValidityCheckingResolution", &ompl::base::SpaceInformation::setStateValidityCheckingResolution)
        .def("getStateValidityCheckingResolution", &ompl::base::SpaceInformation::getStateValidityCheckingResolution)
        .def("allocState", [](const ompl::base::SpaceInformation &si) { 
            ompl::base::State* state = si.allocState();
            return std::shared_ptr<ompl::base::State>(
                state, 
                [&si](ompl::base::State* s) {
                    si.freeState(s);
                }
            );
        }, nb::keep_alive<0, 1>()) // Return value (index 0) keeps self (index 1) alive
        .def("freeState", &ompl::base::SpaceInformation::freeState)
        .def("copyState", &ompl::base::SpaceInformation::copyState)
        .def("cloneState", &ompl::base::SpaceInformation::cloneState)

        .def("allocStateSampler", &ompl::base::SpaceInformation::allocStateSampler)
        .def("allocValidStateSampler", &ompl::base::SpaceInformation::allocValidStateSampler)
        .def("setValidStateSamplerAllocator", &ompl::base::SpaceInformation::setValidStateSamplerAllocator)
        .def("clearValidStateSamplerAllocator", &ompl::base::SpaceInformation::clearValidStateSamplerAllocator)

        .def("getMaximumExtent", &ompl::base::SpaceInformation::getMaximumExtent)
        .def("searchValidNearby", nb::overload_cast<ompl::base::State*, const ompl::base::State*, double, unsigned int>(&ompl::base::SpaceInformation::searchValidNearby, nb::const_))
        .def("searchValidNearby", nb::overload_cast<const ompl::base::ValidStateSamplerPtr&, ompl::base::State*, const ompl::base::State*, double>(&ompl::base::SpaceInformation::searchValidNearby, nb::const_))
        .def("randomBounceMotion", &ompl::base::SpaceInformation::randomBounceMotion)

        .def("checkMotion", nb::overload_cast<const ompl::base::State*, const ompl::base::State*>(&ompl::base::SpaceInformation::checkMotion, nb::const_))
        
       .def("checkMotion", nb::overload_cast<const std::vector<ompl::base::State*>&, unsigned int, unsigned int&>(&ompl::base::SpaceInformation::checkMotion, nb::const_))
        .def("checkMotion", nb::overload_cast<const std::vector<ompl::base::State*>&, unsigned int>(&ompl::base::SpaceInformation::checkMotion, nb::const_))

        .def("getMotionStates", &ompl::base::SpaceInformation::getMotionStates)
        .def("getCheckedMotionCount", &ompl::base::SpaceInformation::getCheckedMotionCount)

        .def("probabilityOfValidState", &ompl::base::SpaceInformation::probabilityOfValidState)
        .def("averageValidMotionLength", &ompl::base::SpaceInformation::averageValidMotionLength)
        .def("samplesPerSecond", &ompl::base::SpaceInformation::samplesPerSecond)
        // Virtual method: printSettings
        .def("printSettings", [](const ompl::base::SpaceInformation &si) { si.printSettings(std::cout); })
        .def("settings", [](const ompl::base::SpaceInformation &si) {
            std::ostringstream oss;
            si.printSettings(oss);
            return oss.str();
        })
        .def("__str__",
             [](const ompl::base::SpaceInformation &si)
             {
                 std::ostringstream oss;
                 si.printSettings(oss);
                 return oss.str();
             })
        .def("__repr__",
             [](const ompl::base::SpaceInformation &si)
             {
                 std::ostringstream oss;
                 si.printSettings(oss);
                 return oss.str();
             })
    // Virtual method: printProperties
    .def("printProperties", [](const ompl::base::SpaceInformation &si) { si.printProperties(std::cout); })
        .def("params", nb::overload_cast<>(&ompl::base::SpaceInformation::params, nb::const_))
        .def("setup", &ompl::base::SpaceInformation::setup)
        .def("isSetup", &ompl::base::SpaceInformation::isSetup);
    // Protected method: setMotionValidator
    // .def("setDefaultMotionValidator", &ompl::base::SpaceInformation::setDefaultMotionValidator);
}
