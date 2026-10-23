#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include "ompl/base/StateSpace.h"
#include "ompl/base/StateSampler.h"
#include <sstream>
#include "PyGC.h"
#include "init.h"
#include "validation.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;

namespace
{
    // The sampler allocator is protected and has no getter.
    struct SpaceAccess : ob::StateSpace
    {
        static const ob::StateSamplerAllocator &samplerAllocator(const ob::StateSpace &space)
        {
            return space.*(&SpaceAccess::ssa_);
        }
    };

    int stateSpaceTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        const auto &space = *nb::inst_ptr<ob::StateSpace>(self);
        if (int rc = gc::visitHeld(SpaceAccess::samplerAllocator(space), visit, arg))
            return rc;
        return ompl::binding::base::visitParams(space.params(), visit, arg);
    }

    int stateSpaceClear(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        auto &space = *nb::inst_ptr<ob::StateSpace>(self);
        if (gc::holdsPython(SpaceAccess::samplerAllocator(space)))
            space.clearStateSamplerAllocator();
        ompl::binding::base::clearParams(space.params());
        return 0;
    }

    ob::State *&substate(const ob::CompoundStateSpace &space, ob::State *state, unsigned int index)
    {
        auto *compound = dynamic_cast<ob::CompoundState *>(state);
        if (compound == nullptr)
            throw nb::type_error("expected a CompoundState");
        if (index >= space.getSubspaceCount())
            throw nb::index_error("subspace index out of range");
        return compound->components[index];
    }

    const PyType_Slot stateSpaceSlots[] = {{Py_tp_traverse, (void *)stateSpaceTraverse},
                                           {Py_tp_clear, (void *)stateSpaceClear},
                                           {0, 0}};
}  // namespace

void ompl::binding::base::init_StateSpace(nb::module_ &m)
{
    nb::class_<ob::StateSpace>(m, "StateSpace", nb::type_slots(stateSpaceSlots))
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
        .def("computeSignature",
             [](const ob::StateSpace &space)
             {
                 std::vector<int> signature;
                 space.computeSignature(signature);
                 return signature;
             })
        .def("cloneState", &ob::StateSpace::cloneState, nb::arg("source"), nb::rv_policy::take_ownership)
        .def("getSerializationLength", &ob::StateSpace::getSerializationLength)
        .def("getValueAddressAtIndex",
             nb::overload_cast<ob::State *, unsigned int>(&ob::StateSpace::getValueAddressAtIndex, nb::const_),
             nb::arg("state"), nb::arg("index"), nb::rv_policy::reference_internal)

        .def("computeLocations", &ob::StateSpace::computeLocations)

        .def(
            "setStateSamplerAllocator", [](ob::StateSpace &space, nb::callable allocator)
            { space.setStateSamplerAllocator(gc::function<ob::StateSamplerAllocator>(std::move(allocator))); },
            nb::arg("allocator"))
        .def("clearStateSamplerAllocator", &ob::StateSpace::clearStateSamplerAllocator)
        .def("allocStateSampler", &ob::StateSpace::allocStateSampler)
        .def("allocDefaultStateSampler", &ob::StateSpace::allocDefaultStateSampler)
        .def("params", nb::overload_cast<>(&ob::StateSpace::params), nb::rv_policy::reference_internal)

        .def("registerProjection", &ob::StateSpace::registerProjection, nb::arg("name"), nb::arg("projection"))
        .def("registerDefaultProjection", &ob::StateSpace::registerDefaultProjection, nb::arg("projection"))
        .def("registerProjections", &ob::StateSpace::registerProjections)
        .def("getProjection", &ob::StateSpace::getProjection, nb::arg("name"))
        .def("getDefaultProjection", &ob::StateSpace::getDefaultProjection)
        .def("hasProjection", &ob::StateSpace::hasProjection, nb::arg("name"))
        .def("hasDefaultProjection", &ob::StateSpace::hasDefaultProjection)
        .def("getRegisteredProjections", &ob::StateSpace::getRegisteredProjections)
        .def(
            "printState", [](const ob::StateSpace &ss, const ob::State *s) { ss.printState(s, std::cout); },
            nb::arg("state"))
        .def("printSettings", [](const ob::StateSpace &ss) { ss.printSettings(std::cout); })
        .def("__repr__",
             [](const ob::StateSpace &ss)
             {
                 std::ostringstream oss;
                 ss.printSettings(oss);
                 return oss.str();
             })
        .def("printProjections", [](const ob::StateSpace &ss) { ss.printProjections(std::cout); })
        .def("sanityChecks", nb::overload_cast<double, double, unsigned int>(&ob::StateSpace::sanityChecks, nb::const_),
             nb::arg("zero"), nb::arg("eps"), nb::arg("flags"))
        .def("Diagram", &ob::StateSpace::Diagram)
        .def("List", &ob::StateSpace::List)
        .def("allocSubspaceStateSampler",
             nb::overload_cast<const ob::StateSpacePtr &>(&ob::StateSpace::allocSubspaceStateSampler, nb::const_),
             nb::arg("subspace"))
        .def("getSubstateAtLocation",
             nb::overload_cast<ob::State *, const ob::StateSpace::SubstateLocation &>(
                 &ob::StateSpace::getSubstateAtLocation, nb::const_),
             nb::arg("state"), nb::arg("loc"), nb::rv_policy::reference_internal)
        .def("getSubstateLocationsByName", &ob::StateSpace::getSubstateLocationsByName,
             nb::rv_policy::reference_internal)
        .def(
            "getCommonSubspaces",
            [](const ob::StateSpace &space, const ob::StateSpacePtr &other)
            {
                std::vector<std::string> subspaces;
                space.getCommonSubspaces(other, subspaces);
                return subspaces;
            },
            nb::arg("other"))
        // Returns [] if setup() (or computeLocations()) has not been called yet. We cannot validate for this like
        // copyFromReals because a zero-dimensional space like EmptyStateSpace returns [] too, and the two are
        // indistinguishable
        .def(
            "copyToReals",
            [](const ob::StateSpace &ss, const ob::State *source)
            {
                std::vector<double> reals;
                ss.copyToReals(reals, source);
                return reals;
            },
            nb::arg("source"))
        .def(
            "copyFromReals",
            [](const ob::StateSpace &ss, ob::State *destination, const std::vector<double> &reals)
            {
                checkRealsSize(ss, reals, "copyFromReals");
                ss.copyFromReals(destination, reals);
            },
            nb::arg("destination"), nb::arg("reals"));

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
        .def(
            "getSubstate", [](const ob::CompoundStateSpace &space, ob::State *state, unsigned int index)
            { return substate(space, state, index); }, nb::arg("state"), nb::arg("index"), nb::rv_policy::reference,
            nb::keep_alive<0, 2>())
        .def(
            "setSubstate",
            [](const ob::CompoundStateSpace &space, ob::State *state, unsigned int index, const ob::State *value)
            { space.getSubspace(index)->copyState(substate(space, state, index), value); }, nb::arg("state"),
            nb::arg("index"), nb::arg("value"))
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
