#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <string>

#include "ompl/base/spaces/HybridTimeStateSpace.h"
#include "ompl/base/StateSpace.h"
#include "base/samplers/OwnerRef.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_HybridTimeStateSpace(nb::module_ &m)
{
    nb::class_<ob::HybridTimeStateSpace::StateType, ob::State>(m, "HybridTimeStateType")
        .def_rw("position", &ob::HybridTimeStateSpace::StateType::position)
        .def_rw("jumps", &ob::HybridTimeStateSpace::StateType::jumps);

    nb::class_<ob::HybridTimeStateSampler, ob::StateSampler, WithOwner<ob::HybridTimeStateSampler>>(
        m, "HybridTimeStateSampler", nb::type_slots(ownerSlots<ob::HybridTimeStateSampler>))
        .def("__init__", initWithOwner<ob::HybridTimeStateSampler, ob::StateSpace>(), nb::arg("space"))
        .def("sampleUniform", &ob::HybridTimeStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::HybridTimeStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::HybridTimeStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::HybridTimeStateSpace, ob::StateSpace>(m, "HybridTimeStateSpace")
        .def(nb::init<>())
        .def("setTimeBounds", &ob::HybridTimeStateSpace::setTimeBounds, nb::arg("lower"), nb::arg("upper"))
        .def("setJumpBounds", &ob::HybridTimeStateSpace::setJumpBounds, nb::arg("lower"), nb::arg("upper"))
        .def("getMinTimeBound", &ob::HybridTimeStateSpace::getMinTimeBound)
        .def("getMaxTimeBound", &ob::HybridTimeStateSpace::getMaxTimeBound)
        .def("getMinJumpsBound", &ob::HybridTimeStateSpace::getMinJumpsBound)
        .def("getMaxJumpBound", &ob::HybridTimeStateSpace::getMaxJumpBound)
        .def("isTimeBounded", &ob::HybridTimeStateSpace::isTimeBounded)
        .def("areJumpsBounded", &ob::HybridTimeStateSpace::areJumpsBounded)
        .def("distance", &ob::HybridTimeStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("interpolate", &ob::HybridTimeStateSpace::interpolate, nb::arg("from"), nb::arg("to"), nb::arg("t"),
             nb::arg("state"))
        .def("allocState", &ob::HybridTimeStateSpace::allocState)
        .def("getDimension", &ob::HybridTimeStateSpace::getDimension)
        .def("getMaximumExtent", &ob::HybridTimeStateSpace::getMaximumExtent)
        .def("getMeasure", &ob::HybridTimeStateSpace::getMeasure)
        .def("enforceBounds", &ob::HybridTimeStateSpace::enforceBounds, nb::arg("state"))
        .def("satisfiesBounds", &ob::HybridTimeStateSpace::satisfiesBounds, nb::arg("state"))
        .def("copyState", &ob::HybridTimeStateSpace::copyState, nb::arg("destination"), nb::arg("source"))
        .def("equalStates", &ob::HybridTimeStateSpace::equalStates, nb::arg("state1"), nb::arg("state2"))
        .def("getSerializationLength", &ob::HybridTimeStateSpace::getSerializationLength)
        .def(
            "serialize",
            [](const ob::HybridTimeStateSpace &space, const ob::State *state)
            {
                std::string buffer(space.getSerializationLength(), '\0');
                space.serialize(buffer.data(), state);
                return nb::bytes(buffer.data(), buffer.size());
            },
            nb::arg("state"))
        .def(
            "deserialize",
            [](const ob::HybridTimeStateSpace &space, ob::State *state, const nb::bytes &serialization)
            {
                if (serialization.size() != space.getSerializationLength())
                    throw nb::value_error(("deserialize() expects " + std::to_string(space.getSerializationLength()) +
                                           " bytes, got " + std::to_string(serialization.size()))
                                              .c_str());
                space.deserialize(state, serialization.c_str());
            },
            nb::arg("state"), nb::arg("serialization"))
        .def("allocDefaultStateSampler", &ob::HybridTimeStateSpace::allocDefaultStateSampler)
        .def("allocStateSampler", &ob::HybridTimeStateSpace::allocStateSampler);
}
