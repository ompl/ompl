#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/base/spaces/SO3StateSpace.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SO3StateSpace(nb::module_& m)
{
    // Bind the SO3StateSampler.
    nb::class_<ompl::base::SO3StateSampler,
               ompl::base::StateSampler>(m, "SO3StateSampler")
        .def(nb::init<const ompl::base::StateSpace*>())
        .def("sampleUniform", &ompl::base::SO3StateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::SO3StateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::SO3StateSampler::sampleGaussian);

    // Bind the inner state type for SO3StateSpace.
    nb::class_<ompl::base::SO3StateSpace::StateType, ompl::base::State>(m, "SO3State")
        .def("setAxisAngle", &ompl::base::SO3StateSpace::StateType::setAxisAngle)
        .def("setIdentity", &ompl::base::SO3StateSpace::StateType::setIdentity)
        .def_rw("x", &ompl::base::SO3StateSpace::StateType::x)
        .def_rw("y", &ompl::base::SO3StateSpace::StateType::y)
        .def_rw("z", &ompl::base::SO3StateSpace::StateType::z)
        .def_rw("w", &ompl::base::SO3StateSpace::StateType::w)
        .def("setAxisAngle", &ompl::base::SO3StateSpace::StateType::setAxisAngle)
        .def("setIdentity", &ompl::base::SO3StateSpace::StateType::setIdentity);

    // Bind the SO3StateSpace class.
    nb::class_<ompl::base::SO3StateSpace,
               ompl::base::StateSpace>(m, "SO3StateSpace")
        .def(nb::init<>())
        .def("norm", &ompl::base::SO3StateSpace::norm)
        .def("getDimension", &ompl::base::SO3StateSpace::getDimension)
        .def("getMaximumExtent", &ompl::base::SO3StateSpace::getMaximumExtent)
        .def("getMeasure", &ompl::base::SO3StateSpace::getMeasure)
        .def("enforceBounds", &ompl::base::SO3StateSpace::enforceBounds)
        .def("satisfiesBounds", &ompl::base::SO3StateSpace::satisfiesBounds)
        .def("getSerializationLength", &ompl::base::SO3StateSpace::getSerializationLength)
        .def("serialize", &ompl::base::SO3StateSpace::serialize)
        .def("deserialize", &ompl::base::SO3StateSpace::deserialize)
        .def("copyState", &ompl::base::SO3StateSpace::copyState)
        .def("distance", &ompl::base::SO3StateSpace::distance)
        .def("equalStates", &ompl::base::SO3StateSpace::equalStates)
        .def("interpolate", &ompl::base::SO3StateSpace::interpolate)
        .def("allocDefaultStateSampler", &ompl::base::SO3StateSpace::allocDefaultStateSampler)
        .def("allocState", &ompl::base::SO3StateSpace::allocState)
        .def("getValueAddressAtIndex", &ompl::base::SO3StateSpace::getValueAddressAtIndex)
        .def("printState", [](const ompl::base::SO3StateSpace &ss, const ompl::base::State *state) {
             ss.printState(state, std::cout);
         }, nb::arg("state"))
        .def("printSettings", [](const ompl::base::SO3StateSpace &ss) {
             ss.printSettings(std::cout);
         })
        .def("registerProjections", &ompl::base::SO3StateSpace::registerProjections);
}