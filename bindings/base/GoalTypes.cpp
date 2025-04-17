#include <nanobind/nanobind.h>
#include "ompl/base/GoalTypes.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_GoalTypes(nb::module_ &m)
{
    nb::enum_<ompl::base::GoalType>(m, "GoalType", "Enumeration of goal types in OMPL")
        .value("GOAL_ANY", ompl::base::GOAL_ANY, "Generic goal type; no specific representation.")
        .value("GOAL_REGION", ompl::base::GOAL_REGION, "Goal defined by a region in state space.")
        .value("GOAL_SAMPLEABLE_REGION", ompl::base::GOAL_SAMPLEABLE_REGION, "A goal defined by a sampleable region.")
        .value("GOAL_STATE", ompl::base::GOAL_STATE, "A goal represented by a single state.")
        .value("GOAL_STATES", ompl::base::GOAL_STATES, "A goal represented by several states.")
        .value("GOAL_LAZY_SAMPLES", ompl::base::GOAL_LAZY_SAMPLES, "A goal where samples are generated lazily.")
        .export_values();
}
