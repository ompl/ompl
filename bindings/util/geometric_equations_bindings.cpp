#include <nanobind/nanobind.h>
#include "ompl/util/GeometricEquations.h"

namespace nb = nanobind;

void init_geometric_equations(nb::module_& m) {
    // Expose geometric equations
    m.def("n_ball_measure", &ompl::nBallMeasure, "Compute the volume of an n-dimensional ball.");
    m.def("unit_n_ball_measure", &ompl::unitNBallMeasure, "Compute the volume of a unit n-dimensional ball.");
    m.def("prolate_hyperspheroid_measure", &ompl::prolateHyperspheroidMeasure, "Compute the volume of a prolate hyperspheroid.");
}