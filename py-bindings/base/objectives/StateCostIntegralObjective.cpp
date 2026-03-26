#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_StateCostIntegralObjective(nb::module_ &m)
{
    struct PyStateCostIntegralObjective : ob::StateCostIntegralObjective
    {
        NB_TRAMPOLINE(ob::StateCostIntegralObjective, 3);

        ob::Cost stateCost(const ob::State *s) const override
        {
            nb::gil_scoped_acquire gil;
            using Ret = ob::Cost;
            nb::object self = nb::find(this);
            if (self.is_valid())
            {
                nb::object cls = self.attr("__class__");
                nb::dict cls_dict(cls.attr("__dict__"));
                if (cls_dict.contains("stateCost"))
                {
                    return nb::cast<Ret>(self.attr("stateCost")(s));
                }
            }
            return ob::StateCostIntegralObjective::stateCost(s);
        }

        ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
        {
            nb::gil_scoped_acquire gil;
            using Ret = ob::Cost;
            nb::object self = nb::find(this);
            if (self.is_valid())
            {
                nb::object cls = self.attr("__class__");
                nb::dict cls_dict(cls.attr("__dict__"));
                if (cls_dict.contains("motionCost"))
                {
                    return nb::cast<Ret>(self.attr("motionCost")(s1, s2));
                }
            }
            return ob::StateCostIntegralObjective::motionCost(s1, s2);
        }

        ob::Cost motionCostBestEstimate(const ob::State *s1, const ob::State *s2) const override
        {
            nb::gil_scoped_acquire gil;
            using Ret = ob::Cost;
            nb::object self = nb::find(this);
            if (self.is_valid())
            {
                nb::object cls = self.attr("__class__");
                nb::dict cls_dict(cls.attr("__dict__"));
                if (cls_dict.contains("motionCostBestEstimate"))
                {
                    return nb::cast<Ret>(self.attr("motionCostBestEstimate")(s1, s2));
                }
            }
            return ob::StateCostIntegralObjective::motionCostBestEstimate(s1, s2);
        }
    };

    nb::class_<ob::StateCostIntegralObjective, ob::OptimizationObjective,
               PyStateCostIntegralObjective /* <-- trampoline */>(m, "StateCostIntegralObjective")
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("enableMotionCostInterpolation"))
        .def("isMotionCostInterpolationEnabled", &ob::StateCostIntegralObjective::isMotionCostInterpolationEnabled);
}
