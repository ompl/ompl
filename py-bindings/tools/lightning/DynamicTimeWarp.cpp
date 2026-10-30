#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/geometric/PathGeometric.h"
#include "ompl/tools/lightning/DynamicTimeWarp.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

void ompl::binding::tools::initLightning_DynamicTimeWarp(nb::module_ &m)
{
    nb::class_<ot::DynamicTimeWarp>(m, "DynamicTimeWarp")
        .def(nb::init<ob::SpaceInformationPtr>(), nb::arg("si"))
        .def("calcDTWDistance", &ot::DynamicTimeWarp::calcDTWDistance, nb::arg("path1"), nb::arg("path2"))
        .def("getPathsScore", &ot::DynamicTimeWarp::getPathsScore, nb::arg("path1"), nb::arg("path2"));
}
