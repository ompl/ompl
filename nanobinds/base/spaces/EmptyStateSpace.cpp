#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/EmptyStateSpace.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_EmptyStateSpace(nb::module_ &m)
{
    // TODO [ob::EmptyStateSpace][TEST]
    nb::class_<ob::EmptyStateSpace, ob::RealVectorStateSpace>(m, "EmptyStateSpace")
        .def(nb::init<>())
        .def("getMeasure", &ob::EmptyStateSpace::getMeasure)
        .def("getDimension", &ob::EmptyStateSpace::getDimension)
        .def("getMaximumExtent", &ob::EmptyStateSpace::getMaximumExtent)
        .def("setup", &ob::EmptyStateSpace::setup);
}
