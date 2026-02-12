#include <nanobind/nanobind.h>
#include "ompl/base/GoalTypes.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::base::init_GoalTypes(nb::module_ &m)
{
    nb::enum_<ompl::base::GoalType>(m, "GoalType")
        .value("GOAL_ANY", ompl::base::GOAL_ANY)
        .value("GOAL_REGION", ompl::base::GOAL_REGION)
        .value("GOAL_SAMPLEABLE_REGION", ompl::base::GOAL_SAMPLEABLE_REGION)
        .value("GOAL_STATE", ompl::base::GOAL_STATE)
        .value("GOAL_STATES", ompl::base::GOAL_STATES)
        .value("GOAL_LAZY_SAMPLES", ompl::base::GOAL_LAZY_SAMPLES)
        .export_values();
}
