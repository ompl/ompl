#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>
#include "ompl/base/spaces/TimeStateSpace.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_TimeStateSpace(nb::module_ &m)
{
    // Bind the TimeStateSampler.
    nb::class_<ompl::base::TimeStateSampler,
               ompl::base::StateSampler>(m, "TimeStateSampler")
        .def(nb::init<const ompl::base::StateSpace*>())
        .def("sampleUniform", &ompl::base::TimeStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::TimeStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::TimeStateSampler::sampleGaussian);

    // Bind the inner state type for TimeStateSpace.
    nb::class_<ompl::base::TimeStateSpace::StateType, ompl::base::State>(m, "TimeState")
        .def_rw("position", &ompl::base::TimeStateSpace::StateType::position);

    // Bind the TimeStateSpace class.
    nb::class_<ompl::base::TimeStateSpace,
               ompl::base::StateSpace>(m, "TimeStateSpace")
        .def(nb::init<>())
        // Set and get bounds (for time, bounds are specified as two doubles)
        .def("setBounds", &ompl::base::TimeStateSpace::setBounds, nb::arg("minTime"), nb::arg("maxTime"))
        .def("getMinTimeBound", &ompl::base::TimeStateSpace::getMinTimeBound)
        .def("getMaxTimeBound", &ompl::base::TimeStateSpace::getMaxTimeBound)
        .def("isBounded", &ompl::base::TimeStateSpace::isBounded)
        .def("enforceBounds", &ompl::base::TimeStateSpace::enforceBounds)
        .def("satisfiesBounds", &ompl::base::TimeStateSpace::satisfiesBounds)
        .def("copyState", &ompl::base::TimeStateSpace::copyState)
        .def("getSerializationLength", &ompl::base::TimeStateSpace::getSerializationLength)
        .def("serialize", &ompl::base::TimeStateSpace::serialize)
        .def("deserialize", &ompl::base::TimeStateSpace::deserialize)
        .def("distance", &ompl::base::TimeStateSpace::distance)
        .def("equalStates", &ompl::base::TimeStateSpace::equalStates)
        .def("interpolate", &ompl::base::TimeStateSpace::interpolate)
        .def("allocDefaultStateSampler", &ompl::base::TimeStateSpace::allocDefaultStateSampler)
        .def("allocState", &ompl::base::TimeStateSpace::allocState)
        .def("getValueAddressAtIndex", &ompl::base::TimeStateSpace::getValueAddressAtIndex)
        .def("printState", [](const ompl::base::TimeStateSpace &ss, const ompl::base::State *state) {
            ss.printState(state, std::cout);
        }, nb::arg("state"))

        .def("printSettings", [](const ompl::base::TimeStateSpace &ss) {
            ss.printSettings(std::cout);
        })
        .def("registerProjections", &ompl::base::TimeStateSpace::registerProjections);
}