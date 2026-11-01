#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/PlannerData.h"
#include "ompl/control/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_PlannerData(nb::module_ &m)
{
    nb::class_<oc::PlannerDataEdgeControl, ob::PlannerDataEdge>(m, "PlannerDataEdgeControl")
        .def("getControl", &oc::PlannerDataEdgeControl::getControl, nb::rv_policy::reference_internal)
        .def("getDuration", &oc::PlannerDataEdgeControl::getDuration);

    nb::class_<oc::PlannerData, ob::PlannerData>(m, "PlannerData")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("getSpaceInformation", &oc::PlannerData::getSpaceInformation)
        .def("hasControls", &oc::PlannerData::hasControls)
        .def("decoupleFromPlanner", &oc::PlannerData::decoupleFromPlanner);
}
