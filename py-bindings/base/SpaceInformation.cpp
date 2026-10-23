#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <sstream>
#include <thread>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/ValidStateSampler.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;

// States handed to Python are owned by their wrapper: ompl::base::State has a protected destructor, so
// nanobind cannot release one on its own. The deleter holds the space that allocated the state.
static std::shared_ptr<ompl::base::State> ownedState(const ompl::base::SpaceInformation &si, ompl::base::State *state)
{
    return {state, [space = si.getStateSpace()](ompl::base::State *s) { space->freeState(s); }};
}

namespace
{
    struct CallableStateValidityChecker : ompl::base::StateValidityChecker, gc::HoldsCallable
    {
        CallableStateValidityChecker(ompl::base::SpaceInformation *si, nb::object fn)
          : ompl::base::StateValidityChecker(si), gc::HoldsCallable(std::move(fn)), owner_(std::this_thread::get_id())
        {
        }

        using ompl::base::StateValidityChecker::isValid;
        bool isValid(const ompl::base::State *state) const override
        {
            if (std::this_thread::get_id() == owner_)
                return gc::call<bool>(callable, state);
            // An exception escaping a planner's or GoalLazySamples' worker thread would std::terminate.
            nb::gil_scoped_acquire gil;
            try
            {
                return gc::call<bool>(callable, state);
            }
            catch (nb::python_error &e)
            {
                e.discard_as_unraisable(callable.get());
            }
            catch (const nb::cast_error &)
            {
                PyErr_SetString(PyExc_TypeError, "a state validity checker must return a bool");
                PyErr_WriteUnraisable(callable.get().ptr());
            }
            return false;
        }

    private:
        std::thread::id owner_;
    };
}  // namespace

int ompl::binding::base::visitSpaceInformation(const ompl::base::SpaceInformation &si, visitproc visit, void *arg)
{
    if (int rc = gc::visitHeld(si.getStateSpace(), visit, arg))
        return rc;
    if (int rc = gc::visitHeld(si.getStateValidityChecker(), visit, arg))
        return rc;
    if (int rc = gc::visitHeld(si.getMotionValidator(), visit, arg))
        return rc;
    for (const auto &entry : si.params().getParams())
        if (int rc = gc::visitHeld(entry.second, visit, arg))
            return rc;
    return 0;
}

void ompl::binding::base::clearSpaceInformation(ompl::base::SpaceInformation &si)
{
    if (gc::holdsPython(si.getStateValidityChecker()))
        si.setStateValidityChecker(ompl::base::StateValidityCheckerPtr());
    if (gc::holdsPython(si.getMotionValidator()))
        si.setMotionValidator(ompl::base::MotionValidatorPtr());
    std::vector<std::string> pythonParams;
    for (const auto &entry : si.params().getParams())
        if (gc::holdsPython(entry.second))
            pythonParams.push_back(entry.first);
    for (const auto &name : pythonParams)
        si.params().remove(name);
}

int space_information_tp_traverse(PyObject *self, visitproc visit, void *arg)
{
    Py_VISIT(Py_TYPE(self));
    if (!nb::inst_ready(self))
        return 0;
    if (int rc = gc::visitDict(self, visit, arg))
        return rc;
    // A view of a C++-owned SpaceInformation does not own what the C++ object holds.
    if (!gc::pythonOwned(self))
        return 0;
    return ompl::binding::base::visitSpaceInformation(*nb::inst_ptr<ompl::base::SpaceInformation>(self), visit, arg);
}

int space_information_tp_clear(PyObject *self)
{
    if (nb::inst_ready(self) && gc::pythonOwned(self))
        ompl::binding::base::clearSpaceInformation(*nb::inst_ptr<ompl::base::SpaceInformation>(self));
    gc::clearDict(self);
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
        .def(
            "setStateValidityChecker",
            [](ompl::base::SpaceInformation &si, const ompl::base::StateValidityCheckerPtr &svc)
            { si.setStateValidityChecker(svc); },
            nb::arg("svc"))
        .def(
            "setStateValidityChecker",
            [](ompl::base::SpaceInformation &si, nb::callable fn)
            { si.setStateValidityChecker(std::make_shared<CallableStateValidityChecker>(&si, std::move(fn))); },
            nb::arg("svc"))
        .def("getStateValidityChecker", &ompl::base::SpaceInformation::getStateValidityChecker)
        .def("setMotionValidator", &ompl::base::SpaceInformation::setMotionValidator, nb::arg("mv"))
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
        .def(
            "randomBounceMotion",
            [](const ompl::base::SpaceInformation &si, const ompl::base::StateSamplerPtr &sampler,
               const ompl::base::State *start, unsigned int steps)
            {
                std::vector<ompl::base::State *> states;
                unsigned int added = 0;
                try
                {
                    added = si.randomBounceMotion(sampler, start, steps, states, true);
                }
                catch (...)
                {
                    for (ompl::base::State *state : states)
                        if (state)
                            si.freeState(state);
                    throw;
                }
                std::vector<std::shared_ptr<ompl::base::State>> owned;
                owned.reserve(added);
                for (std::size_t i = 0; i < states.size(); ++i)
                {
                    if (i < added)
                        owned.push_back(ownedState(si, states[i]));
                    else
                        si.freeState(states[i]);
                }
                return owned;
            },
            nb::arg("sampler"), nb::arg("start"), nb::arg("steps"))
        .def(
            "randomBounceMotion",
            [](const ompl::base::SpaceInformation &si, const ompl::base::StateSamplerPtr &sampler,
               const ompl::base::State *start, unsigned int steps, std::vector<ompl::base::State *> states)
            { return si.randomBounceMotion(sampler, start, steps, states, false); },
            nb::arg("sampler"), nb::arg("start"), nb::arg("steps"), nb::arg("states"))

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
        .def("params", nb::overload_cast<>(&ompl::base::SpaceInformation::params), nb::rv_policy::reference_internal)
        .def("setup", &ompl::base::SpaceInformation::setup)
        .def("isSetup", &ompl::base::SpaceInformation::isSetup);
    // Protected method: setMotionValidator
    // .def("setDefaultMotionValidator", &ompl::base::SpaceInformation::setDefaultMotionValidator);
}
