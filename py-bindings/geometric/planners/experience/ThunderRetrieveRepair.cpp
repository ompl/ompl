#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/experience/ThunderRetrieveRepair.h"
#include "ompl/tools/thunder/ThunderDB.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/geometric/PathGeometric.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

void ompl::binding::geometric::initPlannersExperience_ThunderRetrieveRepair(nb::module_ &m)
{
    nb::class_<og::ThunderRetrieveRepair, ob::Planner>(m, "ThunderRetrieveRepair")
        .def(nb::init<const ob::SpaceInformationPtr &, ot::ThunderDBPtr>(), nb::arg("si"), nb::arg("experienceDB"))
        .def("solve", ompl::binding::solveBinding<og::ThunderRetrieveRepair>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::ThunderRetrieveRepair>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::ThunderRetrieveRepair &self, ob::PlannerData &data)
            { self.getPlannerData(data); }, nb::arg("data"))
        .def("clear", &og::ThunderRetrieveRepair::clear)
        .def("setup", &og::ThunderRetrieveRepair::setup)
        .def("setExperienceDB", &og::ThunderRetrieveRepair::setExperienceDB, nb::arg("experienceDB"))
        .def("setRepairPlanner", &og::ThunderRetrieveRepair::setRepairPlanner, nb::arg("planner"))
        .def("getNearestK", &og::ThunderRetrieveRepair::getNearestK)
        .def("setNearestK", &og::ThunderRetrieveRepair::setNearestK, nb::arg("nearestK"))
        .def("enableSmoothing", &og::ThunderRetrieveRepair::enableSmoothing, nb::arg("enable"))
        .def("getLastRecalledNearestPathChosen", &og::ThunderRetrieveRepair::getLastRecalledNearestPathChosen)
        .def("getLastRecalledNearestPaths", &og::ThunderRetrieveRepair::getLastRecalledNearestPaths)
        .def("getChosenRecallPath", &og::ThunderRetrieveRepair::getChosenRecallPath, nb::rv_policy::reference_internal)
        .def("getRepairPlannerDatas",
             [](const og::ThunderRetrieveRepair &self)
             {
                 std::vector<ob::PlannerDataPtr> data;
                 self.getRepairPlannerDatas(data);
                 return data;
             });
}
