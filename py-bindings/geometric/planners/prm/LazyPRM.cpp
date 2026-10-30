#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ConnectionFilter.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    struct LazyPRMAccess : og::LazyPRM
    {
        static const auto &states(og::LazyPRM &planner)
        {
            return planner.*(&LazyPRMAccess::stateProperty_);
        }

        static const ConnectionFilter &filter(og::LazyPRM &planner)
        {
            return planner.*(&LazyPRMAccess::connectionFilter_);
        }
    };

    using LazyPRMConnectionFilter = ompl::binding::geometric::ConnectionFilterBinding<og::LazyPRM, LazyPRMAccess>;
}  // namespace

void ompl::binding::geometric::initPlannersPrm_LazyPRM(nb::module_ &m)
{
    nb::class_<og::LazyPRM, ob::Planner>(m, "LazyPRM", nb::type_slots(LazyPRMConnectionFilter::slots))
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("starStrategy") = false)
        .def(nb::init<const ob::PlannerData &, bool>(), nb::arg("data"), nb::arg("starStrategy") = false)
        .def("solve", ompl::binding::solveBinding<og::LazyPRM>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::LazyPRM>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::LazyPRM &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::LazyPRM::clear)
        .def("setup", &og::LazyPRM::setup)
        .def("setRange", &og::LazyPRM::setRange, nb::arg("distance"))
        .def("getRange", &og::LazyPRM::getRange)
        .def("setDefaultConnectionStrategy", &og::LazyPRM::setDefaultConnectionStrategy)
        .def("setMaxNearestNeighbors", &og::LazyPRM::setMaxNearestNeighbors, nb::arg("k"))
        .def("setConnectionFilter", &LazyPRMConnectionFilter::set, nb::arg("connectionFilter").none())
        .def("clearQuery", &og::LazyPRM::clearQuery)
        .def("clearValidity", &og::LazyPRM::clearValidity)
        .def("milestoneCount", &og::LazyPRM::milestoneCount)
        .def("edgeCount", &og::LazyPRM::edgeCount);
}
