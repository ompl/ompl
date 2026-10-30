#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/planners/sst/SST.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSst_SST(nb::module_ &m)
{
    nb::class_<oc::SST, ob::Planner>(m, "SST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("setGoalBias", &oc::SST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::SST::getGoalBias)
        .def("setSelectionRadius", &oc::SST::setSelectionRadius, nb::arg("selectionRadius"))
        .def("getSelectionRadius", &oc::SST::getSelectionRadius)
        .def("setPruningRadius", &oc::SST::setPruningRadius, nb::arg("pruningRadius"))
        .def("getPruningRadius", &oc::SST::getPruningRadius);
}
