#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/planners/est/EST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersEst_EST(nb::module_& m)
{
    // TODO [oc::EST][TEST]
    // TODO [oc::EST][MISSING]
    // TAG [oc::EST][Planner]
    nb::class_<ompl::control::EST, ob::Planner>(m, "EST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", &oc::EST::solve, nb::arg("terminationCondition"));
}
