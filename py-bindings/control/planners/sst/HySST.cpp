#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/sst/HySST.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSst_HySST(nb::module_ &m)
{
    nb::class_<oc::HySST, ob::Planner>(m, "HySST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "solve",
            [](oc::HySST &self, nb::object what)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                    return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                if (nb::isinstance<double>(what))
                    return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                throw nb::type_error(
                    "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
            })
        .def(
            "getPlannerData", [](const oc::HySST &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &oc::HySST::clear)
        .def("setup", &oc::HySST::setup)
        .def("setSelectionRadius", &oc::HySST::setSelectionRadius, nb::arg("selectionRadius"))
        .def("getSelectionRadius", &oc::HySST::getSelectionRadius)
        .def("setPruningRadius", &oc::HySST::setPruningRadius, nb::arg("pruningRadius"))
        .def("getPruningRadius", &oc::HySST::getPruningRadius)
        .def("setTm", &oc::HySST::setTm, nb::arg("tM"))
        .def("setFlowStepDuration", &oc::HySST::setFlowStepDuration, nb::arg("duration"))
        .def("setBatchSize", &oc::HySST::setBatchSize, nb::arg("batchSize"));
}
