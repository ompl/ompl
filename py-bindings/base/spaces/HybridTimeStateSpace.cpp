#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/HybridTimeStateSpace.h"
#include "ompl/base/StateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_HybridTimeStateSpace(nb::module_ &m)
{
    nb::class_<ob::HybridTimeStateSpace::StateType, ob::State>(m, "HybridTimeStateType")
        .def_rw("position", &ob::HybridTimeStateSpace::StateType::position)
        .def_rw("jumps", &ob::HybridTimeStateSpace::StateType::jumps);

    nb::class_<ob::HybridTimeStateSampler, ob::StateSampler>(m, "HybridTimeStateSampler")
        .def(nb::init<const ob::StateSpace *>(), nb::arg("space"))
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
        .def("getMaximumExtent", &ob::HybridTimeStateSpace::getMaximumExtent);
}
