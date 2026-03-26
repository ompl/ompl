#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/base/spaces/SO2StateSpace.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SO2StateSpace(nb::module_ &m)
{
    // Bind the SO2 state sampler.
    nb::class_<ompl::base::SO2StateSampler,
               ompl::base::StateSampler>(m, "SO2StateSampler")
        .def(nb::init<const ompl::base::StateSpace*>())
        .def("sampleUniform", &ompl::base::SO2StateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::SO2StateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::SO2StateSampler::sampleGaussian);

    // Bind the inner state type for SO2StateSpace.
    nb::class_<ompl::base::SO2StateSpace::StateType, ompl::base::State>(m, "SO2State")
        .def("setIdentity", &ompl::base::SO2StateSpace::StateType::setIdentity,
             "Set the state to the identity (zero angle)")
        .def_rw("value", &ompl::base::SO2StateSpace::StateType::value);

    nb::class_<ompl::base::SO2StateSpace,
               ompl::base::StateSpace>(m, "SO2StateSpace")
        .def(nb::init<>())
        .def("getDimension", &ompl::base::SO2StateSpace::getDimension)
        .def("getMaximumExtent", &ompl::base::SO2StateSpace::getMaximumExtent)
        .def("getMeasure", &ompl::base::SO2StateSpace::getMeasure)
        .def("enforceBounds", &ompl::base::SO2StateSpace::enforceBounds)
        .def("satisfiesBounds", &ompl::base::SO2StateSpace::satisfiesBounds)
        .def("getSerializationLength", &ompl::base::SO2StateSpace::getSerializationLength)
        .def("serialize", &ompl::base::SO2StateSpace::serialize)
        .def("deserialize", &ompl::base::SO2StateSpace::deserialize)
        .def("copyState", &ompl::base::SO2StateSpace::copyState)
        .def("distance", &ompl::base::SO2StateSpace::distance)
        .def("equalStates", &ompl::base::SO2StateSpace::equalStates)
        .def("interpolate", &ompl::base::SO2StateSpace::interpolate)
        .def("allocDefaultStateSampler", &ompl::base::SO2StateSpace::allocDefaultStateSampler)
        .def("allocState", &ompl::base::SO2StateSpace::allocState)
        .def("printState", [](const ompl::base::SO2StateSpace &ss, const ompl::base::State *state) {
            ss.printState(state, std::cout);
        }, nb::arg("state"))
        .def("printSettings", [](const ompl::base::SO2StateSpace &ss) {
            ss.printSettings(std::cout);
        })
        .def("registerProjections", &ompl::base::SO2StateSpace::registerProjections);
}