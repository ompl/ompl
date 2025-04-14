#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/base/spaces/SO3StateSpace.h"
#include "../init.hh"
#include "common.hh"  // Assumes bind_scoped_state_template is defined here

namespace nb = nanobind;

void ompl::binding::base::initSpaces_SO3StateSpace(nb::module_& m)
{
    // Bind the SO3StateSampler.
    nb::class_<ompl::base::SO3StateSampler,
               ompl::base::StateSampler>(m, "SO3StateSampler")
        .def(nb::init<const ompl::base::StateSpace*>(),
             "Construct a SO3StateSampler from the given state space")
        .def("sampleUniform", &ompl::base::SO3StateSampler::sampleUniform,
             "Sample a state uniformly")
        .def("sampleUniformNear", &ompl::base::SO3StateSampler::sampleUniformNear,
             "Sample a state uniformly near a provided state")
        .def("sampleGaussian", &ompl::base::SO3StateSampler::sampleGaussian,
             "Sample a state using a Gaussian distribution");

    // Bind the inner state type for SO3StateSpace.
    nb::class_<ompl::base::SO3StateSpace::StateType, ompl::base::State>(m, "SO3State")
        .def("setAxisAngle", &ompl::base::SO3StateSpace::StateType::setAxisAngle,
             "Set the state using an axis-angle representation (ax, ay, az, angle)")
        .def("setIdentity", &ompl::base::SO3StateSpace::StateType::setIdentity,
             "Set the state to identity (no rotation)")
        .def_rw("x", &ompl::base::SO3StateSpace::StateType::x,
             "X component of the quaternion")
        .def_rw("y", &ompl::base::SO3StateSpace::StateType::y,
             "Y component of the quaternion")
        .def_rw("z", &ompl::base::SO3StateSpace::StateType::z,
             "Z component of the quaternion")
        .def_rw("w", &ompl::base::SO3StateSpace::StateType::w,
             "W component of the quaternion")
        .def("setAxisAngle", &ompl::base::SO3StateSpace::StateType::setAxisAngle,
             "Set the state using an axis-angle representation (ax, ay, az, angle)")
        .def("setIdentity", &ompl::base::SO3StateSpace::StateType::setIdentity,
             "Set the state to identity (no rotation)");

    // Bind the SO3StateSpace class.
    nb::class_<ompl::base::SO3StateSpace,
               ompl::base::StateSpace>(m, "SO3StateSpace")
        .def(nb::init<>(), "Default constructor for SO3StateSpace")
        .def("norm", &ompl::base::SO3StateSpace::norm, "Compute the norm of a given state")
        .def("getDimension", &ompl::base::SO3StateSpace::getDimension,
             "Return the dimension of the state space")
        .def("getMaximumExtent", &ompl::base::SO3StateSpace::getMaximumExtent,
             "Return the maximum extent of the state space")
        .def("getMeasure", &ompl::base::SO3StateSpace::getMeasure,
             "Return the measure of the state space")
        .def("enforceBounds", &ompl::base::SO3StateSpace::enforceBounds,
             "Enforce state bounds")
        .def("satisfiesBounds", &ompl::base::SO3StateSpace::satisfiesBounds,
             "Check if a state satisfies its bounds")
        .def("getSerializationLength", &ompl::base::SO3StateSpace::getSerializationLength,
             "Return the serialization length of a state")
        .def("serialize", &ompl::base::SO3StateSpace::serialize,
             "Serialize a state to a given buffer")
        .def("deserialize", &ompl::base::SO3StateSpace::deserialize,
             "Deserialize a state from a given buffer")
        .def("copyState", &ompl::base::SO3StateSpace::copyState,
             "Copy one state to another")
        .def("distance", &ompl::base::SO3StateSpace::distance,
             "Compute the distance between two states")
        .def("equalStates", &ompl::base::SO3StateSpace::equalStates,
             "Check if two states are equal")
        .def("interpolate", &ompl::base::SO3StateSpace::interpolate,
             "Interpolate between two states")
        .def("allocDefaultStateSampler", &ompl::base::SO3StateSpace::allocDefaultStateSampler,
             "Allocate the default state sampler")
        .def("allocState", &ompl::base::SO3StateSpace::allocState,
             "Allocate a new state")
        .def("getValueAddressAtIndex", &ompl::base::SO3StateSpace::getValueAddressAtIndex,
             "Return a pointer to the value at the given index")
        .def("printState", [](const ompl::base::SO3StateSpace &ss, const ompl::base::State *state) {
             ss.printState(state, std::cout);
         }, nb::arg("state"), "Print the state")
        .def("printSettings", [](const ompl::base::SO3StateSpace &ss) {
             ss.printSettings(std::cout);
         }, "Print the settings of the state space")
        .def("registerProjections", &ompl::base::SO3StateSpace::registerProjections,
             "Register projections for the state space");
}