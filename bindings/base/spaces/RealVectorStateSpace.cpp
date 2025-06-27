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
    nb::class_<ompl::base::RealVectorStateSampler, ompl::base::StateSampler>(m, "RealVectorStateSampler")
        .def(nb::init<const ompl::base::StateSpace *>())
        .def("sampleUniform", &ompl::base::RealVectorStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::RealVectorStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::RealVectorStateSampler::sampleGaussian);

    nb::class_<ompl::base::RealVectorStateSpace::StateType, ompl::base::State> stateType(m, "RealVectorStateType");
    stateType
        .def("__getitem__",
             [](const ompl::base::RealVectorStateSpace::StateType *s, unsigned int i) { return s->values[i]; }, nb::rv_policy::reference_internal)
        .def("__setitem__",
             [](ompl::base::RealVectorStateSpace::StateType *s, unsigned int i, double v) { s->values[i] = v; });

    // Bind RealVectorStateSpace
    nb::class_<ompl::base::RealVectorStateSpace, ompl::base::StateSpace>(m, "RealVectorStateSpace")
        .def(nb::init<unsigned int>())
        .def(nb::init<>())
        .def("getDimension", &ompl::base::RealVectorStateSpace::getDimension)
        .def("addDimension", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::addDimension))
        .def("addDimension",
             nb::overload_cast<const std::string &, double, double>(&ompl::base::RealVectorStateSpace::addDimension))
        .def("setBounds",
             nb::overload_cast<const ompl::base::RealVectorBounds &>(&ompl::base::RealVectorStateSpace::setBounds))
        .def("setBounds", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::setBounds))
        .def("getBounds", &ompl::base::RealVectorStateSpace::getBounds)
        .def("getDimensionName", &ompl::base::RealVectorStateSpace::getDimensionName)
        .def("getDimensionIndex", &ompl::base::RealVectorStateSpace::getDimensionIndex)
        .def("setDimensionName", &ompl::base::RealVectorStateSpace::setDimensionName)
        .def("getMaximumExtent", &ompl::base::RealVectorStateSpace::getMaximumExtent)
        .def("getMeasure", &ompl::base::RealVectorStateSpace::getMeasure)
        .def("enforceBounds", &ompl::base::RealVectorStateSpace::enforceBounds)
        .def("satisfiesBounds", &ompl::base::RealVectorStateSpace::satisfiesBounds)
        .def("copyState", &ompl::base::RealVectorStateSpace::copyState)
        .def("getSerializationLength", &ompl::base::RealVectorStateSpace::getSerializationLength)
        .def("serialize", &ompl::base::RealVectorStateSpace::serialize)
        .def("deserialize", &ompl::base::RealVectorStateSpace::deserialize)
        .def("distance", &ompl::base::RealVectorStateSpace::distance)
        .def("equalStates", &ompl::base::RealVectorStateSpace::equalStates)
        .def("interpolate", &ompl::base::RealVectorStateSpace::interpolate)
        .def("allocDefaultStateSampler", &ompl::base::RealVectorStateSpace::allocDefaultStateSampler)
        .def("allocState", &ompl::base::RealVectorStateSpace::allocState)
        .def("getValueAddressAtIndex", &ompl::base::RealVectorStateSpace::getValueAddressAtIndex)
        .def(
            "printState", [](const ompl::base::RealVectorStateSpace &space, const ompl::base::State *state)
            { space.printState(state, std::cout); })
        .def(
            "printSettings", [](const ompl::base::RealVectorStateSpace &space) { space.printSettings(std::cout); })
        .def("registerProjections", &ompl::base::RealVectorStateSpace::registerProjections)
        .def("setup", &ompl::base::RealVectorStateSpace::setup);
}