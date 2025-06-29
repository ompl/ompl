#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include "ompl/base/StateSpace.h"
#include <sstream>
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_StateSpace(nb::module_ &m)
{
    nb::class_<ob::StateSpace>(m, "StateSpace")
        .def("isCompound", &ob::StateSpace::isCompound)
        .def("isDiscrete", &ob::StateSpace::isDiscrete)
        .def("isHybrid", &ob::StateSpace::isHybrid)
        .def("isMetricSpace", &ob::StateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::StateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::StateSpace::hasSymmetricInterpolate)
        .def("getName", &ob::StateSpace::getName, nb::rv_policy::reference_internal)
        .def("setName", &ob::StateSpace::setName, nb::arg("name"))
        .def("getType", &ob::StateSpace::getType)
        .def("includes", nb::overload_cast<const ob::StateSpacePtr &>(&ob::StateSpace::includes, nb::const_),
             nb::arg("other"))
        .def("covers", nb::overload_cast<const ob::StateSpacePtr &>(&ob::StateSpace::covers, nb::const_),
             nb::arg("other"))
        .def("getLongestValidSegmentFraction", &ob::StateSpace::getLongestValidSegmentFraction)
        .def("setLongestValidSegmentFraction", &ob::StateSpace::setLongestValidSegmentFraction, nb::arg("fraction"))
        .def("validSegmentCount", &ob::StateSpace::validSegmentCount, nb::arg("state1"), nb::arg("state2"))
        .def("setValidSegmentCountFactor", &ob::StateSpace::setValidSegmentCountFactor, nb::arg("factor"))
        .def("getValidSegmentCountFactor", &ob::StateSpace::getValidSegmentCountFactor)
        .def("getLongestValidSegmentLength", &ob::StateSpace::getLongestValidSegmentLength)
        .def("computeSignature", &ob::StateSpace::computeSignature, nb::arg("signature"))
        .def("cloneState", &ob::StateSpace::cloneState, nb::arg("source"), nb::rv_policy::take_ownership)
        .def("getSerializationLength", &ob::StateSpace::getSerializationLength)
        .def("getValueAddressAtIndex",
             nb::overload_cast<ob::State *, unsigned int>(&ob::StateSpace::getValueAddressAtIndex, nb::const_),
             nb::arg("state"), nb::arg("index"), nb::rv_policy::reference_internal)

        .def("registerProjection", &ob::StateSpace::registerProjection, nb::arg("name"), nb::arg("projection"))
        .def("registerDefaultProjection", &ob::StateSpace::registerDefaultProjection, nb::arg("projection"))
        .def("registerProjections", &ob::StateSpace::registerProjections)
        .def("getProjection", &ob::StateSpace::getProjection, nb::arg("name"), nb::rv_policy::reference_internal)
        .def("getDefaultProjection", &ob::StateSpace::getDefaultProjection, nb::rv_policy::reference_internal)
        .def("hasProjection", &ob::StateSpace::hasProjection, nb::arg("name"))
        .def("hasDefaultProjection", &ob::StateSpace::hasDefaultProjection)
        .def("getRegisteredProjections", &ob::StateSpace::getRegisteredProjections)
        .def(
            "printState", [](const ob::StateSpace &ss, const ob::State *s) { ss.printState(s, std::cout); },
            nb::arg("state"))
        .def("printSettings", [](const ob::StateSpace &ss) { ss.printSettings(std::cout); })
        .def("printProjections", [](const ob::StateSpace &ss) { ss.printProjections(std::cout); })
        .def("sanityChecks", nb::overload_cast<double, double, unsigned int>(&ob::StateSpace::sanityChecks, nb::const_),
             nb::arg("zero"), nb::arg("eps"), nb::arg("flags"))
        .def("Diagram", &ob::StateSpace::Diagram)
        .def("List", &ob::StateSpace::List)
        .def("allocSubspaceStateSampler",
             nb::overload_cast<const ob::StateSpacePtr &>(&ob::StateSpace::allocSubspaceStateSampler, nb::const_),
             nb::arg("subspace"), nb::rv_policy::reference_internal)
        .def("getSubstateAtLocation",
             nb::overload_cast<ob::State *, const ob::StateSpace::SubstateLocation &>(
                 &ob::StateSpace::getSubstateAtLocation, nb::const_),
             nb::arg("state"), nb::arg("loc"), nb::rv_policy::reference_internal)
        .def("getSubstateLocationsByName",
             &ob::StateSpace::getSubstateLocationsByName,
             nb::rv_policy::reference_internal)
        .def("getCommonSubspaces",
             nb::overload_cast<const ob::StateSpacePtr &, std::vector<std::string> &>(
                 &ob::StateSpace::getCommonSubspaces, nb::const_),
             nb::arg("other"), nb::arg("subspaces"))
        .def("copyToReals", &ob::StateSpace::copyToReals, nb::arg("reals"), nb::arg("source"))
        .def("copyFromReals", &ob::StateSpace::copyFromReals, nb::arg("destination"), nb::arg("reals"));

    nb::class_<ob::CompoundStateSpace, ob::StateSpace>(m, "CompoundStateSpace")
        .def(nb::init<>())
        .def(nb::init<const std::vector<ob::StateSpacePtr> &, const std::vector<double> &>())

        .def("isCompound", &ob::CompoundStateSpace::isCompound)
        .def("isHybrid", &ob::CompoundStateSpace::isHybrid)
        .def("printState",
             [](const ob::CompoundStateSpace &space, const ob::State *state) { space.printState(state, std::cout); })
        .def("printSettings", [](const ob::CompoundStateSpace &space) { space.printSettings(std::cout); })
        .def("computeLocations", &ob::CompoundStateSpace::computeLocations)
        .def("setup", &ob::CompoundStateSpace::setup)

        // Management of subspaces
        .def("addSubspace", &ob::CompoundStateSpace::addSubspace)
        .def("getSubspaceCount", &ob::CompoundStateSpace::getSubspaceCount)
        .def("getSubspace", nb::overload_cast<unsigned int>(&ob::CompoundStateSpace::getSubspace, nb::const_))
        .def("getSubspace", nb::overload_cast<const std::string &>(&ob::CompoundStateSpace::getSubspace, nb::const_))
        .def("hasSubspace", &ob::CompoundStateSpace::hasSubspace)
        .def("getSubspaceWeight",
             nb::overload_cast<unsigned int>(&ob::CompoundStateSpace::getSubspaceWeight, nb::const_))
        .def("getSubspaceWeight",
             nb::overload_cast<const std::string &>(&ob::CompoundStateSpace::getSubspaceWeight, nb::const_))
        .def("setSubspaceWeight", nb::overload_cast<unsigned int, double>(&ob::CompoundStateSpace::setSubspaceWeight))
        .def("setSubspaceWeight",
             nb::overload_cast<const std::string &, double>(&ob::CompoundStateSpace::setSubspaceWeight))
        .def("getSubspaces", &ob::CompoundStateSpace::getSubspaces)
        .def("getSubspaceWeights", &ob::CompoundStateSpace::getSubspaceWeights)
        .def("isLocked", &ob::CompoundStateSpace::isLocked)
        .def("lock", &ob::CompoundStateSpace::lock)

        .def("allocSubspaceStateSampler", &ob::CompoundStateSpace::allocSubspaceStateSampler)

        // State space functionality
        .def("getDimension", &ob::CompoundStateSpace::getDimension)
        .def("getMaximumExtent", &ob::CompoundStateSpace::getMaximumExtent)
        .def("getMeasure", &ob::CompoundStateSpace::getMeasure)
        .def("enforceBounds", &ob::CompoundStateSpace::enforceBounds)
        .def("satisfiesBounds", &ob::CompoundStateSpace::satisfiesBounds)
        .def("copyState", &ob::CompoundStateSpace::copyState)
        .def("getSerializationLength", &ob::CompoundStateSpace::getSerializationLength)
        .def("serialize", &ob::CompoundStateSpace::serialize)
        .def("deserialize", &ob::CompoundStateSpace::deserialize)
        .def("distance", &ob::CompoundStateSpace::distance)
        .def("setLongestValidSegmentFraction", &ob::CompoundStateSpace::setLongestValidSegmentFraction)
        .def("validSegmentCount", &ob::CompoundStateSpace::validSegmentCount)
        .def("equalStates", &ob::CompoundStateSpace::equalStates)
        .def("interpolate", &ob::CompoundStateSpace::interpolate)
        .def("allocDefaultStateSampler", &ob::CompoundStateSpace::allocDefaultStateSampler)
        .def("allocState", &ob::CompoundStateSpace::allocState)
        .def("freeState", &ob::CompoundStateSpace::freeState)
        .def("getValueAddressAtIndex", &ob::CompoundStateSpace::getValueAddressAtIndex);
}
