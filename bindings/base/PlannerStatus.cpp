#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include "ompl/base/PlannerStatus.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_PlannerStatus(nb::module_ &m)
{
    auto ps =
        nb::class_<ompl::base::PlannerStatus>(m, "PlannerStatus")
            .def(nb::init<>())
            .def("getStatus", [](const ompl::base::PlannerStatus &s)
                 { return static_cast<ompl::base::PlannerStatus::StatusType>(s); })
            .def(nb::init<ompl::base::PlannerStatus::StatusType>())
            .def(nb::init<bool, bool>())
            .def_prop_ro("StatusType", [](const ompl::base::PlannerStatus &s)
                         { return static_cast<ompl::base::PlannerStatus::StatusType>(s); })
            .def("asString", &ompl::base::PlannerStatus::asString)

            .def("__bool__", [](const ompl::base::PlannerStatus &self) -> bool { return static_cast<bool>(self); })
            .def("__int__", [](const ompl::base::PlannerStatus &self) -> int
                 { return static_cast<ompl::base::PlannerStatus::StatusType>(self); })
            .def("__repr__", [](const ompl::base::PlannerStatus &self) { return self.asString(); });

    nb::enum_<ompl::base::PlannerStatus::StatusType>(ps, "PlannerStatusType")
        .value("UNKNOWN", ompl::base::PlannerStatus::UNKNOWN)
        .value("INVALID_START", ompl::base::PlannerStatus::INVALID_START)
        .value("INVALID_GOAL", ompl::base::PlannerStatus::INVALID_GOAL)
        .value("UNRECOGNIZED_GOAL_TYPE", ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE)
        .value("TIMEOUT", ompl::base::PlannerStatus::TIMEOUT)
        .value("APPROXIMATE_SOLUTION", ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
        .value("EXACT_SOLUTION", ompl::base::PlannerStatus::EXACT_SOLUTION)
        .value("CRASH", ompl::base::PlannerStatus::CRASH)
        .value("ABORT", ompl::base::PlannerStatus::ABORT)
        .value("INFEASIBLE", ompl::base::PlannerStatus::INFEASIBLE)
        .value("TYPE_COUNT", ompl::base::PlannerStatus::TYPE_COUNT)
        .export_values();
}
