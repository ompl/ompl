// bindings/base/goals/GoalRegion.cpp
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <sstream>

#include "ompl/base/goals/GoalRegion.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalRegion(nb::module_ &m)
{
    struct PyGoalRegion : ob::GoalRegion
    {
        NB_TRAMPOLINE(ob::GoalRegion, 1);

        double distanceGoal(const ob::State *st) const override
        {
            NB_OVERRIDE_PURE(distanceGoal, st);
        }
    };

    nb::class_<ob::GoalRegion, ob::Goal,
               PyGoalRegion /* <-- trampoline */>(m, "GoalRegion")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"),
             "Construct a GoalRegion helper for a workspace region")
        // both overloads of isSatisfied
        .def("isSatisfied", nb::overload_cast<const ob::State *>(&ob::GoalRegion::isSatisfied, nb::const_),
             nb::arg("state"), "Check if a state satisfies the goal region")
        .def("isSatisfied", nb::overload_cast<const ob::State *, double *>(&ob::GoalRegion::isSatisfied, nb::const_),
             nb::arg("state"), nb::arg("distance"), "Check if a state satisfies the goal region, and return distance")
        .def("setThreshold", &ob::GoalRegion::setThreshold, nb::arg("threshold"), "Set the goal‐region threshold")
        .def("getThreshold", &ob::GoalRegion::getThreshold, "Get the goal‐region threshold")
        // friendly print() returning a string
        .def(
            "print",
            [](const ob::GoalRegion &self)
            {
                self.print(std::cout);
            },
            "Return a string representation of this GoalRegion");
}
