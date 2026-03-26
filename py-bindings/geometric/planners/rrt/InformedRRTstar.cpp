#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_InformedRRTstar(nb::module_ &m)
{
    nb::class_<og::InformedRRTstar, og::RRTstar>(m, "InformedRRTstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));
}

