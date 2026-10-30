#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/PRM.h"
#include "ConnectionFilter.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

namespace
{
    struct PRMAccess : PRM
    {
        static const auto &states(PRM &planner)
        {
            return planner.*(&PRMAccess::stateProperty_);
        }

        static const ConnectionFilter &filter(PRM &planner)
        {
            return planner.*(&PRMAccess::connectionFilter_);
        }
    };

    using PRMConnectionFilter = ompl::binding::geometric::ConnectionFilterBinding<PRM, PRMAccess>;
}  // namespace

void ompl::binding::geometric::initPlannersPrm_PRM(nb::module_ &m)
{
    // TAG [og::PRM][Planner]
    nb::class_<PRM, Planner>(m, "PRM", nb::type_slots(PRMConnectionFilter::slots))
        .def(nb::init<const SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("starStrategy") = false)
        .def("setMaxNearestNeighbors", &PRM::setMaxNearestNeighbors, nb::arg("k"))
        .def("getMaxNearestNeighbors", &PRM::getMaxNearestNeighbors)
        .def("setDefaultConnectionStrategy", &PRM::setDefaultConnectionStrategy)
        .def("setConnectionFilter", &PRMConnectionFilter::set, nb::arg("connectionFilter").none())
        .def(
            "constructRoadmap",
            [](PRM &self, nb::handle what)
            {
                const PlannerTerminationCondition ptc = ompl::binding::toTerminationCondition(what);
                nb::gil_scoped_release release;
                self.constructRoadmap(ptc);
            },
            nb::arg("what"))
        .def("growRoadmap", nb::overload_cast<double>(&PRM::growRoadmap), nb::arg("growTime"),
             nb::call_guard<nb::gil_scoped_release>())
        .def("growRoadmapPtc", nb::overload_cast<const PlannerTerminationCondition &>(&PRM::growRoadmap),
             nb::arg("ptc"), nb::call_guard<nb::gil_scoped_release>())
        .def("expandRoadmap", nb::overload_cast<double>(&PRM::expandRoadmap), nb::arg("expandTime"),
             nb::call_guard<nb::gil_scoped_release>())
        .def("expandRoadmapPtc", nb::overload_cast<const PlannerTerminationCondition &>(&PRM::expandRoadmap),
             nb::arg("ptc"), nb::call_guard<nb::gil_scoped_release>())
        .def("setup", &PRM::setup)
        .def("clear", &PRM::clear)
        .def("clearQuery", &PRM::clearQuery)
        .def("solve", ompl::binding::solveBinding<PRM>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<PRM>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def("getPlannerData", &PRM::getPlannerData, nb::arg("data"))
        .def("milestoneCount", &PRM::milestoneCount)
        .def("edgeCount", &PRM::edgeCount);
}
