#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/special/SphereStateSpace.h"
#include "ompl/base/StateSpace.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_SphereStateSpace(nb::module_ &m)
{
    nb::class_<ob::SphereStateSpace::StateType, ob::CompoundState>(m, "SphereStateType")
        .def("getTheta", &ob::SphereStateSpace::StateType::getTheta)
        .def("getPhi", &ob::SphereStateSpace::StateType::getPhi)
        .def("setTheta", &ob::SphereStateSpace::StateType::setTheta, nb::arg("theta"))
        .def("setPhi", &ob::SphereStateSpace::StateType::setPhi, nb::arg("phi"))
        .def("setThetaPhi", &ob::SphereStateSpace::StateType::setThetaPhi, nb::arg("theta"), nb::arg("phi"));

    nb::class_<ob::SphereStateSampler, ob::StateSampler>(m, "SphereStateSampler")
        .def(nb::init<const ob::StateSpace *>(), nb::arg("space"))
        .def("sampleUniform", &ob::SphereStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::SphereStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::SphereStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::SphereStateSpace, ob::CompoundStateSpace>(m, "SphereStateSpace")
        .def(nb::init<double>(), nb::arg("radius") = 1.0)
        .def("getMeasure", &ob::SphereStateSpace::getMeasure)
        .def("distance", &ob::SphereStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("allocState", &ob::SphereStateSpace::allocState)
        .def("getMaximumExtent", &ob::SphereStateSpace::getMaximumExtent)
        .def(
            "toVector",
            [](const ob::SphereStateSpace &space, const ob::State *state)
            {
                const auto v = space.toVector(state);
                return std::vector<float>{v.x(), v.y(), v.z()};
            },
            nb::arg("state"));
}
