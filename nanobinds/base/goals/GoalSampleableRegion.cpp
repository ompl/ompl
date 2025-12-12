#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/pair.h>

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalSampleableRegion(nb::module_ &m)
{
    struct PyGoalSampleableRegion : ob::GoalSampleableRegion
    {
        NB_TRAMPOLINE(ob::GoalSampleableRegion, 4);

        void sampleGoal(ob::State *st) const override
        {
            NB_OVERRIDE_PURE(sampleGoal, st);
        }

        unsigned int maxSampleCount() const override
        {
            NB_OVERRIDE_PURE(maxSampleCount);
        }

        bool couldSample() const override
        {
            NB_OVERRIDE(couldSample);
        }

        double distanceGoal(const ob::State *st) const override
        {
            NB_OVERRIDE_PURE(distanceGoal, st);
        }
    };
    nb::class_<ob::GoalSampleableRegion, ob::GoalRegion, PyGoalSampleableRegion /* <-- trampoline */>(m, "GoalSampleabl"
                                                                                                         "eRegion")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        // inherit everything from GoalRegion
        .def("getThreshold", &ob::GoalRegion::getThreshold)
        .def("setThreshold", &ob::GoalRegion::setThreshold, nb::arg("threshold"))

        // optional override in Python
        .def("couldSample", &ob::GoalSampleableRegion::couldSample, "Return true if you could sample another goal.")

        // from GoalRegion / Goal
        .def("isSatisfied", nb::overload_cast<const ob::State *>(&ob::Goal::isSatisfied, nb::const_))
        .def("isSatisfiedWithDistance",
             [](const ob::Goal &g, const ob::State *st)
             {
                 double d;
                 bool ok = g.isSatisfied(st, &d);
                 return std::make_pair(ok, d);
             })
        .def("print", [](const ob::Goal &g) { g.print(std::cout); });
}
