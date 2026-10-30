#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/sst/HySST.h"
#include "../HybridPlanner.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace hybrid = ompl::binding::control::hybrid;

void ompl::binding::control::initPlannersSst_HySST(nb::module_ &m)
{
    using Motion = oc::HySST::Motion;

    auto cls = hybrid::bindPlanner<oc::HySST>(m, "HySST")
                   .def("setSelectionRadius", &oc::HySST::setSelectionRadius, nb::arg("selectionRadius"))
                   .def("getSelectionRadius", &oc::HySST::getSelectionRadius)
                   .def("setPruningRadius", &oc::HySST::setPruningRadius, nb::arg("pruningRadius"))
                   .def("getPruningRadius", &oc::HySST::getPruningRadius)
                   .def("setBatchSize", &oc::HySST::setBatchSize, nb::arg("batchSize"));
    hybrid::bindMotion<Motion>(cls)
        .def("getState", &Motion::getState, nb::rv_policy::reference)
        .def("getParent", &Motion::getParent, nb::rv_policy::reference)
        .def_ro("accCost", &Motion::accCost_)
        .def_ro("numChildren", &Motion::numChildren_)
        .def_ro("inactive", &Motion::inactive_);
}
