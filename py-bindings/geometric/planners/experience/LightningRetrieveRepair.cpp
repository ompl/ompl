#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/experience/LightningRetrieveRepair.h"
#include "ompl/tools/lightning/LightningDB.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

void ompl::binding::geometric::initPlannersExperience_LightningRetrieveRepair(nb::module_ &m)
{
    nb::class_<og::LightningRetrieveRepair, ob::Planner>(m, "LightningRetrieveRepair")
        .def(nb::init<const ob::SpaceInformationPtr &, ot::LightningDBPtr>(), nb::arg("si"), nb::arg("experienceDB"))
        .def("solve",
             [](og::LightningRetrieveRepair &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 if (nb::isinstance<double>(what))
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 throw nb::type_error(
                     "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
             })
        .def(
            "getPlannerData", [](const og::LightningRetrieveRepair &self, ob::PlannerData &data)
            { self.getPlannerData(data); }, nb::arg("data"))
        .def("clear", &og::LightningRetrieveRepair::clear)
        .def("setup", &og::LightningRetrieveRepair::setup)
        .def("setLightningDB", &og::LightningRetrieveRepair::setLightningDB, nb::arg("experienceDB"))
        .def("setRepairPlanner", &og::LightningRetrieveRepair::setRepairPlanner, nb::arg("planner"))
        .def("getNumNearestSolutions", &og::LightningRetrieveRepair::getNumNearestSolutions)
        .def("setNumNearestSolutions", &og::LightningRetrieveRepair::setNumNearestSolutions, nb::arg("nearestK"))
        .def("getLastRecalledNearestPathChosen", &og::LightningRetrieveRepair::getLastRecalledNearestPathChosen);
}
