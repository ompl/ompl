#include <nanobind/nanobind.h>

#include "ompl/util/GeometricEquations.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_GeometricEquations(nb::module_ &m)
{
    m.def("nBallMeasure", &ompl::nBallMeasure, nb::arg("N"), nb::arg("r"));
    m.def("unitNBallMeasure", &ompl::unitNBallMeasure, nb::arg("N"));
    m.def("prolateHyperspheroidMeasure", &ompl::prolateHyperspheroidMeasure, nb::arg("N"), nb::arg("dFoci"),
          nb::arg("dTransverse"));
}
