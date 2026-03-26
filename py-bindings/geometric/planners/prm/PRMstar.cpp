#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/PRMstar.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

void ompl::binding::geometric::initPlannersPrm_PRMstar(nb::module_& m)
{
    // TAG [og::PRMstar][Planner]
    nb::class_<PRMstar, PRM>(m, "PRMstar")
        .def(nb::init<const SpaceInformationPtr&>(),
             nb::arg("si"));
}

