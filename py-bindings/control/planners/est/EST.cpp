#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/planners/est/EST.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersEst_EST(nb::module_ &m)
{
    nb::class_<ompl::control::EST, ob::Planner>(m, "EST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", &oc::EST::solve, nb::arg("terminationCondition"));
}
