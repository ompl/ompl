#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include "ompl/util/RandomNumbers.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_RandomNumbers(nb::module_ &m)
{
    nb::class_<ompl::RNG>(m, "RNG")
        .def(nb::init<>())
        .def(nb::init<std::uint_fast32_t>(), nb::arg("localSeed"))
        .def("uniform01", &ompl::RNG::uniform01)
        .def("uniformReal", &ompl::RNG::uniformReal, nb::arg("lower_bound"), nb::arg("upper_bound"))
        .def("uniformInt", &ompl::RNG::uniformInt, nb::arg("lower_bound"), nb::arg("upper_bound"))
        .def("uniformBool", &ompl::RNG::uniformBool)
        .def("gaussian01", &ompl::RNG::gaussian01)
        .def("gaussian", &ompl::RNG::gaussian, nb::arg("mean"), nb::arg("stddev"))
        .def("halfNormalReal", &ompl::RNG::halfNormalReal, nb::arg("r_min"), nb::arg("r_max"), nb::arg("focus") = 3.0)
        .def("halfNormalInt", &ompl::RNG::halfNormalInt, nb::arg("r_min"), nb::arg("r_max"), nb::arg("focus") = 3.0)
        // quaternion and eulerRPY take raw arrays - wrap them to return vectors
        .def("quaternion",
             [](ompl::RNG &rng)
             {
                 std::vector<double> q(4);
                 rng.quaternion(q.data());
                 return q;
             })
        .def("eulerRPY",
             [](ompl::RNG &rng)
             {
                 std::vector<double> rpy(3);
                 rng.eulerRPY(rpy.data());
                 return rpy;
             })
        .def("uniformNormalVector", &ompl::RNG::uniformNormalVector, nb::arg("v"))
        .def("uniformInBall", &ompl::RNG::uniformInBall, nb::arg("r"), nb::arg("v"))
        .def("setLocalSeed", &ompl::RNG::setLocalSeed, nb::arg("localSeed"))
        .def("getLocalSeed", &ompl::RNG::getLocalSeed)
        .def_static("getSeed", &ompl::RNG::getSeed)
        .def_static("setSeed", &ompl::RNG::setSeed, nb::arg("seed"));
}
