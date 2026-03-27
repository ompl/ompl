#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_PathLengthOptimizationObjective(nb::module_ &m)
{
    struct PyPathLengthOptimizationObjective : ob::PathLengthOptimizationObjective
    {
        NB_TRAMPOLINE(ob::PathLengthOptimizationObjective, 5);

        // Optional override
        ob::Cost stateCost(const ob::State *s) const override
        {
            NB_OVERRIDE(stateCost, s);
        }

        // Optional override
        ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCost, s1, s2);
        }

        // Optional override
        ob::Cost motionCostHeuristic(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCostHeuristic, s1, s2);
        }

        // Optional override
        ob::Cost motionCostBestEstimate(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCostBestEstimate, s1, s2);
        }
    };

    nb::class_<ob::PathLengthOptimizationObjective, ob::OptimizationObjective,
               PyPathLengthOptimizationObjective /* <-- trampoline */>(m, "PathLengthOptimizationObjective")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));
}
