#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSpace.h"

namespace nb = nanobind;

void initSpaceInformation(nb::module_& m) {    
    nb::class_<ompl::base::SpaceInformation>(m, "SpaceInformation")
        .def(nb::init<const std::shared_ptr<ompl::base::StateSpace>&>())
        .def("setStateValidityChecker", 
             nb::overload_cast<const std::function<bool(const ompl::base::State*)>&>
             (&ompl::base::SpaceInformation::setStateValidityChecker))
        .def("isValid", &ompl::base::SpaceInformation::isValid)
        .def("setup", &ompl::base::SpaceInformation::setup)
        .def("getStateSpace", &ompl::base::SpaceInformation::getStateSpace)
        .def("allocState", &ompl::base::SpaceInformation::allocState)
        .def("freeState", &ompl::base::SpaceInformation::freeState)
        .def("getMotionValidator", nb::overload_cast<>(&ompl::base::SpaceInformation::getMotionValidator, nb::const_))
        .def("setValidStateSamplerAllocator", &ompl::base::SpaceInformation::setValidStateSamplerAllocator)
        .def("clearValidStateSamplerAllocator", &ompl::base::SpaceInformation::clearValidStateSamplerAllocator);
}