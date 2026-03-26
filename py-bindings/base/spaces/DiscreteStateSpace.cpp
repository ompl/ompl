#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "../init.h"
namespace nb = nanobind;

void ompl::binding::base::initSpaces_DiscreteStateSpace(nb::module_ &m)
{
    // Bind the discrete state sampler.
    nb::class_<ompl::base::DiscreteStateSampler, ompl::base::StateSampler>(m, "DiscreteStateSampler")
        .def(nb::init<const ompl::base::StateSpace *>())
        .def("sampleUniform", &ompl::base::DiscreteStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::DiscreteStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::DiscreteStateSampler::sampleGaussian);

    // Bind the inner state type for DiscreteStateSpace.
    nb::class_<ompl::base::DiscreteStateSpace::StateType, ompl::base::State>(m, "DiscreteState")
        .def_rw("value", &ompl::base::DiscreteStateSpace::StateType::value);

    // Bind the DiscreteStateSpace.
    nb::class_<ompl::base::DiscreteStateSpace, ompl::base::StateSpace>(m, "DiscreteStateSpace")
        .def(nb::init<int, int>(), nb::arg("lowerBound"), nb::arg("upperBound"))
        .def("isDiscrete", &ompl::base::DiscreteStateSpace::isDiscrete)
        .def("getDimension", &ompl::base::DiscreteStateSpace::getDimension)
        .def("getMaximumExtent", &ompl::base::DiscreteStateSpace::getMaximumExtent)
        .def("getMeasure", &ompl::base::DiscreteStateSpace::getMeasure)
        .def("enforceBounds", &ompl::base::DiscreteStateSpace::enforceBounds)
        .def("satisfiesBounds", &ompl::base::DiscreteStateSpace::satisfiesBounds)
        .def("getSerializationLength", &ompl::base::DiscreteStateSpace::getSerializationLength)
        .def("serialize", &ompl::base::DiscreteStateSpace::serialize)
        .def("deserialize", &ompl::base::DiscreteStateSpace::deserialize)
        .def("copyState", &ompl::base::DiscreteStateSpace::copyState)
        .def("distance", &ompl::base::DiscreteStateSpace::distance)
        .def("equalStates", &ompl::base::DiscreteStateSpace::equalStates)
        .def("interpolate", &ompl::base::DiscreteStateSpace::interpolate)
        .def("allocDefaultStateSampler", &ompl::base::DiscreteStateSpace::allocDefaultStateSampler)
        .def("allocState", &ompl::base::DiscreteStateSpace::allocState)
        .def(
            "printState", [](const ompl::base::DiscreteStateSpace &ds, const ompl::base::State *state)
            { ds.printState(state, std::cout); }, nb::arg("state"), "Print the state to standard output")
        .def(
            "printSettings", [](const ompl::base::DiscreteStateSpace &ds) { ds.printSettings(std::cout); },
            "Print the settings of the state space to standard output")
        .def("registerProjections", &ompl::base::DiscreteStateSpace::registerProjections)
        .def("getStateCount", &ompl::base::DiscreteStateSpace::getStateCount)
        .def("getLowerBound", &ompl::base::DiscreteStateSpace::getLowerBound)
        .def("getUpperBound", &ompl::base::DiscreteStateSpace::getUpperBound)
        .def("setBounds", &ompl::base::DiscreteStateSpace::setBounds)
        .def("setup", &ompl::base::DiscreteStateSpace::setup);
}