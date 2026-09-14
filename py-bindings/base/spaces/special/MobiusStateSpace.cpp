#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/special/MobiusStateSpace.h"
#include "ompl/base/StateSpace.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesSpecial_MobiusStateSpace(nb::module_ &m)
{
    nb::class_<ob::MobiusStateSpace::StateType, ob::CompoundState>(m, "MobiusStateType")
        .def("getU", &ob::MobiusStateSpace::StateType::getU)
        .def("getV", &ob::MobiusStateSpace::StateType::getV)
        .def("setU", &ob::MobiusStateSpace::StateType::setU, nb::arg("u"))
        .def("setV", &ob::MobiusStateSpace::StateType::setV, nb::arg("v"))
        .def("setUV", &ob::MobiusStateSpace::StateType::setUV, nb::arg("u"), nb::arg("v"));

    nb::class_<ob::MobiusStateSpace, ob::CompoundStateSpace>(m, "MobiusStateSpace")
        .def(nb::init<double, double>(), nb::arg("intervalMax") = 1.0, nb::arg("radius") = 1.0)
        .def("distance", &ob::MobiusStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("interpolate", &ob::MobiusStateSpace::interpolate, nb::arg("from"), nb::arg("to"), nb::arg("t"),
             nb::arg("state"))
        .def("allocState", &ob::MobiusStateSpace::allocState)
        .def(
            "toVector",
            [](const ob::MobiusStateSpace &space, const ob::State *state)
            {
                const auto v = space.toVector(state);
                return std::vector<float>{v.x(), v.y(), v.z()};
            },
            nb::arg("state"));
}
