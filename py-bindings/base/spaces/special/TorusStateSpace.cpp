#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/special/TorusStateSpace.h"
#include "ompl/base/StateSpace.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_TorusStateSpace(nb::module_ &m)
{
    nb::class_<ob::TorusStateSpace::StateType, ob::CompoundState>(m, "TorusStateType")
        .def("getS1", &ob::TorusStateSpace::StateType::getS1)
        .def("getS2", &ob::TorusStateSpace::StateType::getS2)
        .def("setS1", &ob::TorusStateSpace::StateType::setS1, nb::arg("s"))
        .def("setS2", &ob::TorusStateSpace::StateType::setS2, nb::arg("s"))
        .def("setS1S2", &ob::TorusStateSpace::StateType::setS1S2, nb::arg("s1"), nb::arg("s2"));

    nb::class_<ob::TorusStateSampler, ob::StateSampler>(m, "TorusStateSampler")
        .def(nb::init<const ob::StateSpace *>(), nb::arg("space"))
        .def("sampleUniform", &ob::TorusStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::TorusStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::TorusStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::TorusStateSpace, ob::CompoundStateSpace>(m, "TorusStateSpace")
        .def(nb::init<double, double>(), nb::arg("majorRadius") = 1.0, nb::arg("minorRadius") = 0.5)
        .def("distance", &ob::TorusStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("allocState", &ob::TorusStateSpace::allocState)
        .def("getMajorRadius", &ob::TorusStateSpace::getMajorRadius)
        .def("getMinorRadius", &ob::TorusStateSpace::getMinorRadius)
        .def(
            "toVector",
            [](const ob::TorusStateSpace &space, const ob::State *state)
            {
                const auto v = space.toVector(state);
                return std::vector<float>{v.x(), v.y(), v.z()};
            },
            nb::arg("state"));
}
