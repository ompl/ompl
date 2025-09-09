#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../init.hh"
#include "common.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
void ompl::binding::base::initSpaces_RealVectorStateSpace(nb::module_ &m)
{
    nb::class_<ob::RealVectorStateSampler, ob::StateSampler>(m, "RealVectorStateSampler")
        .def(nb::init<const ob::StateSpace *>())
        .def("sampleUniform", &ob::RealVectorStateSampler::sampleUniform)
        .def("sampleUniformNear", &ob::RealVectorStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ob::RealVectorStateSampler::sampleGaussian);

    nb::class_<ob::RealVectorStateSpace::StateType, ob::State> stateType(m, "RealVectorStateType");
    stateType
        .def("__getitem__",
             [](const ob::RealVectorStateSpace::StateType *s, unsigned int i) { return s->values[i]; }, nb::rv_policy::reference_internal)
        .def("__setitem__",
             [](ob::RealVectorStateSpace::StateType *s, unsigned int i, double v) { s->values[i] = v; });

    // Bind RealVectorStateSpace
    nb::class_<ob::RealVectorStateSpace, ob::StateSpace>(m, "RealVectorStateSpace")
        .def(nb::init<unsigned int>())
        .def(nb::init<>())
        .def("getDimension", &ob::RealVectorStateSpace::getDimension)
        .def("addDimension", nb::overload_cast<double, double>(&ob::RealVectorStateSpace::addDimension))
        .def("addDimension",
             nb::overload_cast<const std::string &, double, double>(&ob::RealVectorStateSpace::addDimension))
        .def("setBounds",
             nb::overload_cast<const ob::RealVectorBounds &>(&ob::RealVectorStateSpace::setBounds))
        .def("setBounds", nb::overload_cast<double, double>(&ob::RealVectorStateSpace::setBounds))
        .def("getBounds", &ob::RealVectorStateSpace::getBounds)
        .def("getDimensionName", &ob::RealVectorStateSpace::getDimensionName)
        .def("getDimensionIndex", &ob::RealVectorStateSpace::getDimensionIndex)
        .def("setDimensionName", &ob::RealVectorStateSpace::setDimensionName)
        .def("getMaximumExtent", &ob::RealVectorStateSpace::getMaximumExtent)
        .def("getMeasure", &ob::RealVectorStateSpace::getMeasure)
        .def("enforceBounds", &ob::RealVectorStateSpace::enforceBounds)
        .def("satisfiesBounds", &ob::RealVectorStateSpace::satisfiesBounds)
        .def("copyState", &ob::RealVectorStateSpace::copyState)
        .def("getSerializationLength", &ob::RealVectorStateSpace::getSerializationLength)
        .def("serialize", &ob::RealVectorStateSpace::serialize)
        .def("deserialize", &ob::RealVectorStateSpace::deserialize)
        .def("distance", &ob::RealVectorStateSpace::distance)
        .def("equalStates", &ob::RealVectorStateSpace::equalStates)
        .def("interpolate", &ob::RealVectorStateSpace::interpolate)
        .def("allocDefaultStateSampler", &ob::RealVectorStateSpace::allocDefaultStateSampler)
        .def("allocState", &ob::RealVectorStateSpace::allocState)
        .def("getValueAddressAtIndex", &ob::RealVectorStateSpace::getValueAddressAtIndex)
        .def(
            "printState", [](const ob::RealVectorStateSpace &space, const ob::State *state)
            { space.printState(state, std::cout); })
        .def(
            "printSettings", [](const ob::RealVectorStateSpace &space) { space.printSettings(std::cout); })
        .def("registerProjections", &ob::RealVectorStateSpace::registerProjections)
        .def("setup", &ob::RealVectorStateSpace::setup);
}