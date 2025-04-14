#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>
#include "ompl/base/spaces/TimeStateSpace.h"
#include "../init.hh"
#include "common.hh"  // Assumes bind_scoped_state_template is defined here

namespace nb = nanobind;

void ompl::binding::base::initSpaces_TimeStateSpace(nb::module_ &m)
{
    // Bind the TimeStateSampler.
    nb::class_<ompl::base::TimeStateSampler,
               ompl::base::StateSampler>(m, "TimeStateSampler")
        .def(nb::init<const ompl::base::StateSpace*>(),
             "Construct a TimeStateSampler from the given state space")
        .def("sampleUniform", &ompl::base::TimeStateSampler::sampleUniform,
             "Sample a state uniformly")
        .def("sampleUniformNear", &ompl::base::TimeStateSampler::sampleUniformNear,
             "Sample a state uniformly near a given state")
        .def("sampleGaussian", &ompl::base::TimeStateSampler::sampleGaussian,
             "Sample a state using a Gaussian distribution");

    // Bind the inner state type for TimeStateSpace.
    nb::class_<ompl::base::TimeStateSpace::StateType, ompl::base::State>(m, "TimeState")
        .def_rw("position", &ompl::base::TimeStateSpace::StateType::position,
             "The time coordinate");

    // Bind the TimeStateSpace class.
    nb::class_<ompl::base::TimeStateSpace,
               ompl::base::StateSpace>(m, "TimeStateSpace")
        .def(nb::init<>(), "Default constructor for TimeStateSpace")
        // Set and get bounds (for time, bounds are specified as two doubles)
        .def("setBounds", &ompl::base::TimeStateSpace::setBounds,
             "Set the time bounds", nb::arg("minTime"), nb::arg("maxTime"))
        .def("getMinTimeBound", &ompl::base::TimeStateSpace::getMinTimeBound,
             "Return the minimum time bound")
        .def("getMaxTimeBound", &ompl::base::TimeStateSpace::getMaxTimeBound,
             "Return the maximum time bound")
        .def("isBounded", &ompl::base::TimeStateSpace::isBounded,
             "Return whether the time state space is bounded")
        .def("enforceBounds", &ompl::base::TimeStateSpace::enforceBounds,
             "Enforce time bounds on a state")
        .def("satisfiesBounds", &ompl::base::TimeStateSpace::satisfiesBounds,
             "Check if a state satisfies the time bounds")
        .def("copyState", &ompl::base::TimeStateSpace::copyState,
             "Copy a state")
        .def("getSerializationLength", &ompl::base::TimeStateSpace::getSerializationLength,
             "Return the serialization length of a state")
        .def("serialize", &ompl::base::TimeStateSpace::serialize,
             "Serialize a state to a buffer")
        .def("deserialize", &ompl::base::TimeStateSpace::deserialize,
             "Deserialize a state from a buffer")
        .def("distance", &ompl::base::TimeStateSpace::distance,
             "Compute the distance between two states")
        .def("equalStates", &ompl::base::TimeStateSpace::equalStates,
             "Check if two states are equal")
        .def("interpolate", &ompl::base::TimeStateSpace::interpolate,
             "Interpolate between two states")
        .def("allocDefaultStateSampler", &ompl::base::TimeStateSpace::allocDefaultStateSampler,
             "Allocate the default state sampler")
        .def("allocState", &ompl::base::TimeStateSpace::allocState,
             "Allocate a new state")
        .def("getValueAddressAtIndex", &ompl::base::TimeStateSpace::getValueAddressAtIndex,
             "Return a pointer to the value at the given index")
        .def("printState", [](const ompl::base::TimeStateSpace &ss, const ompl::base::State *state) {
            ss.printState(state, std::cout);
        }, nb::arg("state"), "Print the state to standard output")

        .def("printSettings", [](const ompl::base::TimeStateSpace &ss) {
            ss.printSettings(std::cout);
        }, "Print the settings of the state space to standard output")
        .def("registerProjections", &ompl::base::TimeStateSpace::registerProjections,
             "Register projections for the state space");
}