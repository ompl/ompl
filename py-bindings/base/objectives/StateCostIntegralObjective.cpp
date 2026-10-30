#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_StateCostIntegralObjective(nb::module_ &m)
{
    struct PyStateCostIntegralObjective : ob::StateCostIntegralObjective
    {
        NB_TRAMPOLINE(ob::StateCostIntegralObjective, 4);

        ob::Cost stateCost(const ob::State *s) const override
        {
            NB_OVERRIDE(stateCost, s);
        }

        ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCost, s1, s2);
        }

        ob::Cost motionCostBestEstimate(const ob::State *s1, const ob::State *s2) const override
        {
            NB_OVERRIDE(motionCostBestEstimate, s1, s2);
        }

        ob::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn,
                                                         unsigned int maxNumberCalls) const override
        {
            NB_OVERRIDE(allocInformedStateSampler, probDefn, maxNumberCalls);
        }
    };

    nb::class_<ob::StateCostIntegralObjective, ob::OptimizationObjective,
               PyStateCostIntegralObjective /* <-- trampoline */>(m, "StateCostIntegralObjective")
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"),
             nb::arg("enableMotionCostInterpolation") = false)
        .def("isMotionCostInterpolationEnabled", &ob::StateCostIntegralObjective::isMotionCostInterpolationEnabled);
}
