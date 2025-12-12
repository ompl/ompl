#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <sstream>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/ValidStateSampler.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_SpaceInformation(nb::module_& m)
{
    nb::class_<ompl::base::SpaceInformation>(m, "SpaceInformation")
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
            nb::overload_cast<const std::function<bool(const ompl::base::State*)>&>
            (&ompl::base::SpaceInformation::setStateValidityChecker))
        .def("getStateValidityChecker", &ompl::base::SpaceInformation::getStateValidityChecker)
        .def("setMotionValidator", &ompl::base::SpaceInformation::setMotionValidator)
        .def("getMotionValidator", nb::overload_cast<>(&ompl::base::SpaceInformation::getMotionValidator, nb::const_))
        .def("setStateValidityCheckingResolution", &ompl::base::SpaceInformation::setStateValidityCheckingResolution)
        .def("getStateValidityCheckingResolution", &ompl::base::SpaceInformation::getStateValidityCheckingResolution)
        .def("allocState", &ompl::base::SpaceInformation::allocState)
        .def("allocStates", &ompl::base::SpaceInformation::allocStates)
        .def("freeState", &ompl::base::SpaceInformation::freeState)
        .def("freeStates", &ompl::base::SpaceInformation::freeStates)
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
        
        // TODO: this function has compile error
        // .def("checkMotion", nb::overload_cast<const ompl::base::State*, const ompl::base::State*, std::pair<ompl::base::State*, double>&>(&ompl::base::SpaceInformation::checkMotion, nb::const_))
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
        // Virtual method: printProperties
        .def("printProperties", [](const ompl::base::SpaceInformation &si) { si.printProperties(std::cout); })
        .def("params", nb::overload_cast<>(&ompl::base::SpaceInformation::params, nb::const_))
        .def("setup", &ompl::base::SpaceInformation::setup)
        .def("isSetup", &ompl::base::SpaceInformation::isSetup);
        // Protected method: setMotionValidator
        // .def("setDefaultMotionValidator", &ompl::base::SpaceInformation::setDefaultMotionValidator);
}
