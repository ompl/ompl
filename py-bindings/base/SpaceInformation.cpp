#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <sstream>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/ValidStateSampler.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;

static PyObject **get_dict_ptr(PyObject *obj)
{
    return gc::dictPtr(obj);
}

// States handed to Python are owned by their wrapper: ompl::base::State has a protected destructor, so
// nanobind cannot release one on its own. The deleter holds the space that allocated the state.
static std::shared_ptr<ompl::base::State> ownedState(const ompl::base::SpaceInformation &si, ompl::base::State *state)
{
    return {state, [space = si.getStateSpace()](ompl::base::State *s) { space->freeState(s); }};
}

int space_information_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;

    // The validity checker is stashed in __dict__ by its setters, so visiting the dict covers it. Reporting it
    // a second time from the C++ side would claim more references than exist and the collector would keep the
    // cycle alive as if something outside held it.
    PyObject **dictptr = get_dict_ptr(self);
    if (dictptr && *dictptr)
    {
        Py_VISIT(*dictptr);
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
            [](ompl::base::SpaceInformation &si, nb::callable func) {
                // The strong reference lives in __dict__ where tp_clear can drop it; handing OMPL a
                // std::function built by nanobind's caster would instead keep the callable alive with a
                // Py_INCREF the collector cannot see, and the cycle could never be broken.
                nb::handle self = nb::find(si);
                nb::object keeper = gc::keeper(self, func);
                si.setStateValidityChecker(
                    [fn = nb::handle(func), keeper](const ompl::base::State *state)
                    {
                        // PRM and LazyPRM call this from their solution-checking thread.
                        nb::gil_scoped_acquire gil;
                        return nb::cast<bool>(fn(state));
                    });
                // Only now: publishing first would drop the previous callback while OMPL still borrows it.
                if (self.is_valid()) nb::setattr(self, "_svc", func);
            },
            nb::arg("svc"))
        .def("setStateValidityChecker",
            [](ompl::base::SpaceInformation &si, ompl::base::StateValidityChecker *checker) {
                // An owning shared_ptr would carry a py_deleter reference to the checker that the collector
                // cannot see, making it a GC root and pinning everything it reaches. __dict__ holds it
                // instead, so tp_traverse reports it exactly once and tp_clear can drop it.
                gc::installBorrowed<ompl::base::StateValidityCheckerPtr>(
                    nb::find(si), "_svc", checker,
                    [&si](const ompl::base::StateValidityCheckerPtr &svc) { si.setStateValidityChecker(svc); });
            },
            nb::arg("svc"))
        .def("getStateValidityChecker", &ompl::base::SpaceInformation::getStateValidityChecker)
        .def(
            "setMotionValidator",
            [](ompl::base::SpaceInformation &si, ompl::base::MotionValidator *mv)
            {
                gc::installBorrowed<ompl::base::MotionValidatorPtr>(
                    nb::find(si), "_mv", mv,
                    [&si](const ompl::base::MotionValidatorPtr &v) { si.setMotionValidator(v); });
            },
            nb::arg("mv"))
        .def("getMotionValidator", nb::overload_cast<>(&ompl::base::SpaceInformation::getMotionValidator, nb::const_))
        .def("setStateValidityCheckingResolution", &ompl::base::SpaceInformation::setStateValidityCheckingResolution)
        .def("getStateValidityCheckingResolution", &ompl::base::SpaceInformation::getStateValidityCheckingResolution)
        .def("allocState", [](const ompl::base::SpaceInformation &si) { return ownedState(si, si.allocState()); })
        .def("copyState", &ompl::base::SpaceInformation::copyState)
        .def(
            "cloneState",
            [](const ompl::base::SpaceInformation &si, const ompl::base::State *source)
            { return ownedState(si, si.cloneState(source)); },
            nb::arg("source"))

        .def("allocStateSampler", &ompl::base::SpaceInformation::allocStateSampler)
        .def("allocValidStateSampler", &ompl::base::SpaceInformation::allocValidStateSampler)
        .def("setValidStateSamplerAllocator", &ompl::base::SpaceInformation::setValidStateSamplerAllocator)
        .def("clearValidStateSamplerAllocator", &ompl::base::SpaceInformation::clearValidStateSamplerAllocator)

        .def("getMaximumExtent", &ompl::base::SpaceInformation::getMaximumExtent)
        .def("searchValidNearby", nb::overload_cast<ompl::base::State*, const ompl::base::State*, double, unsigned int>(&ompl::base::SpaceInformation::searchValidNearby, nb::const_))
        .def("searchValidNearby", nb::overload_cast<const ompl::base::ValidStateSamplerPtr&, ompl::base::State*, const ompl::base::State*, double>(&ompl::base::SpaceInformation::searchValidNearby, nb::const_))
        .def("randomBounceMotion", &ompl::base::SpaceInformation::randomBounceMotion)

        .def("checkMotion", nb::overload_cast<const ompl::base::State*, const ompl::base::State*>(&ompl::base::SpaceInformation::checkMotion, nb::const_))

        .def(
            "checkMotionFirstInvalid",
            [](const ompl::base::SpaceInformation &si, const std::vector<ompl::base::State *> &states,
               unsigned int count)
            {
                unsigned int firstInvalid = 0;
                bool valid = si.checkMotion(states, count, firstInvalid);
                return std::make_pair(valid, firstInvalid);
            },
            nb::arg("states"), nb::arg("count"))
        .def("checkMotion", nb::overload_cast<const std::vector<ompl::base::State*>&, unsigned int>(&ompl::base::SpaceInformation::checkMotion, nb::const_))

        .def(
            "getMotionStates",
            [](const ompl::base::SpaceInformation &si, const ompl::base::State *s1, const ompl::base::State *s2,
               unsigned int count, bool endpoints)
            {
                std::vector<ompl::base::State *> states;
                unsigned int added = si.getMotionStates(s1, s2, states, count, endpoints, true);
                std::vector<std::shared_ptr<ompl::base::State>> owned;
                owned.reserve(added);
                for (unsigned int i = 0; i < added; ++i)
                    owned.push_back(ownedState(si, states[i]));
                return owned;
            },
            nb::arg("s1"), nb::arg("s2"), nb::arg("count"), nb::arg("endpoints") = true)
        .def("getCheckedMotionCount", &ompl::base::SpaceInformation::getCheckedMotionCount)

        .def("probabilityOfValidState", &ompl::base::SpaceInformation::probabilityOfValidState)
        .def("averageValidMotionLength", &ompl::base::SpaceInformation::averageValidMotionLength)
        .def(
            "samplesPerSecond",
            [](const ompl::base::SpaceInformation &si, unsigned int attempts)
            {
                double uniform = 0.0, near = 0.0, gaussian = 0.0;
                si.samplesPerSecond(uniform, near, gaussian, attempts);
                return std::make_tuple(uniform, near, gaussian);
            },
            nb::arg("attempts"))
        // Virtual method: printSettings
        .def("printSettings", [](const ompl::base::SpaceInformation &si) { si.printSettings(std::cout); })
        .def("settings", [](const ompl::base::SpaceInformation &si) {
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
