#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/EmptyStateSpace.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_EmptyStateSpace(nb::module_ &m)
{
    nb::class_<ob::EmptyStateSpace, ob::RealVectorStateSpace>(m, "EmptyStateSpace")
        .def(nb::init<>(), "Construct an EmptyStateSpace with 0 dimensions")
        .def("getMeasure", &ob::EmptyStateSpace::getMeasure, "Return the measure of the space (always 0)")
        .def("getDimension", &ob::EmptyStateSpace::getDimension, "Return the dimension (always 0)")
        .def("getMaximumExtent", &ob::EmptyStateSpace::getMaximumExtent, "Return the maximum extent (always 0)")
        .def("setup", &ob::EmptyStateSpace::setup, "No-op setup override for EmptyStateSpace");
}
