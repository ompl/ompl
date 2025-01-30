#include <nanobind/nanobind.h>
#include "ompl/util/RandomNumbers.h"

namespace nb = nanobind;

void init_random_numbers(nb::module_& m) {
    // Expose RNG class
    nb::class_<ompl::RNG>(m, "RNG")
        .def(nb::init<>())
        .def("uniform01", &ompl::RNG::uniform01)
        .def("uniform_real", &ompl::RNG::uniformReal)
        .def("uniform_int", &ompl::RNG::uniformInt)
        .def("uniform_bool", &ompl::RNG::uniformBool)
        .def("gaussian01", &ompl::RNG::gaussian01)
        .def("gaussian", &ompl::RNG::gaussian)
        .def("half_normal_real", &ompl::RNG::halfNormalReal)
        .def("half_normal_int", &ompl::RNG::halfNormalInt)
        .def("quaternion", &ompl::RNG::quaternion)
        .def("euler_rpy", &ompl::RNG::eulerRPY)
        .def("set_seed", &ompl::RNG::setLocalSeed)
        .def("get_seed", &ompl::RNG::getLocalSeed);
}