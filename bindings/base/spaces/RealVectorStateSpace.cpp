#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../init.hh"
#include "common.hh"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_RealVectorStateSpace(nb::module_ &m)
{
    nb::class_<ompl::base::RealVectorStateSampler, ompl::base::StateSampler>(m, "RealVectorStateSampler",
                                                                             "State sampler for the R^n state space")
        .def(nb::init<const ompl::base::StateSpace *>())
        .def("sampleUniform", &ompl::base::RealVectorStateSampler::sampleUniform, "Sample state uniformly")
        .def("sampleUniformNear", &ompl::base::RealVectorStateSampler::sampleUniformNear,
             "Sample state uniformly near another state")
        .def("sampleGaussian", &ompl::base::RealVectorStateSampler::sampleGaussian,
             "Sample state from Gaussian distribution");

    nb::class_<ompl::base::RealVectorStateSpace::StateType, ompl::base::State> stateType(m, "RealVectorStateType",
                                                                                         "State in R^n represented as "
                                                                                         "a vector of doubles");
    stateType
        .def("__getitem__",
             [](const ompl::base::RealVectorStateSpace::StateType *s, unsigned int i) { return s->values[i]; })
        .def("__setitem__",
             [](ompl::base::RealVectorStateSpace::StateType *s, unsigned int i, double v) { s->values[i] = v; });

    auto scopedState = bind_scoped_state_template<ompl::base::RealVectorStateSpace>(m, "RealVectorScopedState",
                                                                                    "Scoped state for the "
                                                                                    "RealVectorStateSpace");

    scopedState
        .def("__getitem__", [](const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &self, unsigned int i)
             { return self->values[i]; })
        .def("__setitem__", [](ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &self, unsigned int i,
                               double v) { self->values[i] = v; });

    // Bind RealVectorStateSpace
    nb::class_<ompl::base::RealVectorStateSpace, ompl::base::StateSpace>(m, "RealVectorStateSpace",
                                                                         "A state space representing R^n with L2 norm "
                                                                         "distance")
        .def(nb::init<unsigned int>(), "Constructor. dim represents dimension of R^n")
        .def("getDimension", &ompl::base::RealVectorStateSpace::getDimension, "Get the dimension of the space")
        .def("addDimension", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::addDimension),
             "Add dimension with bounds [minBound, maxBound]")
        .def("addDimension",
             nb::overload_cast<const std::string &, double, double>(&ompl::base::RealVectorStateSpace::addDimension),
             "Add named dimension with bounds")
        .def("setBounds",
             nb::overload_cast<const ompl::base::RealVectorBounds &>(&ompl::base::RealVectorStateSpace::setBounds),
             "Set bounds for all dimensions")
        .def("setBounds", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::setBounds),
             "Set uniform bounds [low, high] for all dimensions")
        .def("getBounds", &ompl::base::RealVectorStateSpace::getBounds, "Get bounds of state space")
        .def("getDimensionName", &ompl::base::RealVectorStateSpace::getDimensionName, "Get name of dimension at index")
        .def("getDimensionIndex", &ompl::base::RealVectorStateSpace::getDimensionIndex, "Get index of named dimension")
        .def("setDimensionName", &ompl::base::RealVectorStateSpace::setDimensionName, "Set name for dimension at index")
        .def("getMaximumExtent", &ompl::base::RealVectorStateSpace::getMaximumExtent,
             "Get maximum possible distance between states")
        .def("getMeasure", &ompl::base::RealVectorStateSpace::getMeasure, "Get measure of space (volume)")
        .def("enforceBounds", &ompl::base::RealVectorStateSpace::enforceBounds, "Project state inside bounds")
        .def("satisfiesBounds", &ompl::base::RealVectorStateSpace::satisfiesBounds, "Check if state is within bounds")
        .def("copyState", &ompl::base::RealVectorStateSpace::copyState, "Copy state to another")
        .def("getSerializationLength", &ompl::base::RealVectorStateSpace::getSerializationLength,
             "Get length needed for state serialization")
        .def("serialize", &ompl::base::RealVectorStateSpace::serialize, "Serialize state to binary")
        .def("deserialize", &ompl::base::RealVectorStateSpace::deserialize, "Deserialize state from binary")
        .def("distance", &ompl::base::RealVectorStateSpace::distance, "Compute distance between states")
        .def("equalStates", &ompl::base::RealVectorStateSpace::equalStates, "Check if states are equal")
        .def("interpolate", &ompl::base::RealVectorStateSpace::interpolate, "Interpolate between states")
        .def("allocDefaultStateSampler", &ompl::base::RealVectorStateSpace::allocDefaultStateSampler,
             "Allocate default state sampler")
        .def("allocState", &ompl::base::RealVectorStateSpace::allocState, "Allocate new state")
        .def("getValueAddressAtIndex", &ompl::base::RealVectorStateSpace::getValueAddressAtIndex,
             "Get pointer to value at index in state")
        .def(
            "printState", [](const ompl::base::RealVectorStateSpace &space, const ompl::base::State *state)
            { space.printState(state, std::cout); }, "Return string representation of state")
        .def(
            "printSettings", [](const ompl::base::RealVectorStateSpace &space) { space.printSettings(std::cout); },
            "Return string representation of space settings")
        .def("registerProjections", &ompl::base::RealVectorStateSpace::registerProjections,
             "Register default projections")
        .def("setup", &ompl::base::RealVectorStateSpace::setup, "Perform space setup");
}