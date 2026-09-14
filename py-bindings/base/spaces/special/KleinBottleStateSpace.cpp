#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/special/KleinBottleStateSpace.h"
#include "ompl/base/StateSpace.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_KleinBottleStateSpace(nb::module_ &m)
{
    nb::class_<ob::KleinBottleStateSpace::StateType, ob::CompoundState>(m, "KleinBottleStateType")
        .def("getU", &ob::KleinBottleStateSpace::StateType::getU)
        .def("getV", &ob::KleinBottleStateSpace::StateType::getV)
        .def("setU", &ob::KleinBottleStateSpace::StateType::setU, nb::arg("u"))
        .def("setV", &ob::KleinBottleStateSpace::StateType::setV, nb::arg("v"))
        .def("setUV", &ob::KleinBottleStateSpace::StateType::setUV, nb::arg("u"), nb::arg("v"));

    nb::class_<ob::KleinBottleStateSampler, ob::StateSampler>(m, "KleinBottleStateSampler")
        .def(nb::init<const ob::StateSpace *>(), nb::arg("space"))
        .def("sampleUniform", &ob::KleinBottleStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::KleinBottleStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::KleinBottleStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::KleinBottleStateSpace, ob::CompoundStateSpace>(m, "KleinBottleStateSpace")
        .def(nb::init<>())
        .def("distance", &ob::KleinBottleStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("interpolate", &ob::KleinBottleStateSpace::interpolate, nb::arg("from"), nb::arg("to"), nb::arg("t"),
             nb::arg("state"))
        .def("allocState", &ob::KleinBottleStateSpace::allocState)
        .def(
            "toVector",
            [](const ob::KleinBottleStateSpace &space, const ob::State *state)
            {
                const auto v = space.toVector(state);
                return std::vector<float>{v.x(), v.y(), v.z()};
            },
            nb::arg("state"));
}
