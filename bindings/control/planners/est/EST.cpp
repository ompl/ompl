#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/planners/est/EST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::initPlannersEst_EST(nb::module_& m)
{
    // TODO [ob::EST][TEST]
    // TODO [ob::EST][MISSING]
    nb::class_<ompl::control::EST, ompl::control::Planner>(m, "EST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", &oc::EST::solve, nb::arg("terminationCondition"))
}
