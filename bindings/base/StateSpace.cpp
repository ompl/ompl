#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include "ompl/base/StateSpace.h"
#include <sstream>
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_StateSpace(nb::module_& m)
{
    nb::class_<ompl::base::StateSpace>(m, "StateSpace");
    
    nb::class_<ompl::base::CompoundStateSpace, ompl::base::StateSpace>(m, "CompoundStateSpace")
    .def(nb::init<>())
    .def(nb::init<const std::vector<ompl::base::StateSpacePtr>&, const std::vector<double>&>())
    
    .def("isCompound", &ompl::base::CompoundStateSpace::isCompound)
    .def("isHybrid", &ompl::base::CompoundStateSpace::isHybrid)
    .def("printState", [](const ompl::base::CompoundStateSpace &space, const ompl::base::State *state) { space.printState(state, std::cout); })
    .def("printSettings", [](const ompl::base::CompoundStateSpace &space) { space.printSettings(std::cout); })
    .def("computeLocations", &ompl::base::CompoundStateSpace::computeLocations)
    .def("setup", &ompl::base::CompoundStateSpace::setup)

    // Management of subspaces
    .def("addSubspace", &ompl::base::CompoundStateSpace::addSubspace)
    .def("getSubspaceCount", &ompl::base::CompoundStateSpace::getSubspaceCount)
    .def("getSubspace", nb::overload_cast<unsigned int>(&ompl::base::CompoundStateSpace::getSubspace, nb::const_))
    .def("getSubspace", nb::overload_cast<const std::string&>(&ompl::base::CompoundStateSpace::getSubspace, nb::const_))
    .def("hasSubspace", &ompl::base::CompoundStateSpace::hasSubspace)
    .def("getSubspaceWeight", nb::overload_cast<unsigned int>(&ompl::base::CompoundStateSpace::getSubspaceWeight, nb::const_))
    .def("getSubspaceWeight", nb::overload_cast<const std::string&>(&ompl::base::CompoundStateSpace::getSubspaceWeight, nb::const_))
    .def("setSubspaceWeight", nb::overload_cast<unsigned int, double>(&ompl::base::CompoundStateSpace::setSubspaceWeight))
    .def("setSubspaceWeight", nb::overload_cast<const std::string&, double>(&ompl::base::CompoundStateSpace::setSubspaceWeight))
    .def("getSubspaces", &ompl::base::CompoundStateSpace::getSubspaces)
    .def("getSubspaceWeights", &ompl::base::CompoundStateSpace::getSubspaceWeights)
    .def("isLocked", &ompl::base::CompoundStateSpace::isLocked)
    .def("lock", &ompl::base::CompoundStateSpace::lock)

    .def("allocSubspaceStateSampler", &ompl::base::CompoundStateSpace::allocSubspaceStateSampler)

    // State space functionality
    .def("getDimension", &ompl::base::CompoundStateSpace::getDimension)
    .def("getMaximumExtent", &ompl::base::CompoundStateSpace::getMaximumExtent)
    .def("getMeasure", &ompl::base::CompoundStateSpace::getMeasure)
    .def("enforceBounds", &ompl::base::CompoundStateSpace::enforceBounds)
    .def("satisfiesBounds", &ompl::base::CompoundStateSpace::satisfiesBounds)
    .def("copyState", &ompl::base::CompoundStateSpace::copyState)
    .def("getSerializationLength", &ompl::base::CompoundStateSpace::getSerializationLength)
    .def("serialize", &ompl::base::CompoundStateSpace::serialize)
    .def("deserialize", &ompl::base::CompoundStateSpace::deserialize)
    .def("distance", &ompl::base::CompoundStateSpace::distance)
    .def("setLongestValidSegmentFraction", &ompl::base::CompoundStateSpace::setLongestValidSegmentFraction)
    .def("validSegmentCount", &ompl::base::CompoundStateSpace::validSegmentCount)
    .def("equalStates", &ompl::base::CompoundStateSpace::equalStates)
    .def("interpolate", &ompl::base::CompoundStateSpace::interpolate)
    .def("allocDefaultStateSampler", &ompl::base::CompoundStateSpace::allocDefaultStateSampler)
    .def("allocState", &ompl::base::CompoundStateSpace::allocState)
    .def("freeState", &ompl::base::CompoundStateSpace::freeState)
    .def("getValueAddressAtIndex", &ompl::base::CompoundStateSpace::getValueAddressAtIndex);
}
