// bindings/base/goals/GoalRegion.cpp
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <sstream>

#include "ompl/base/goals/GoalRegion.h"
#include "../init.h"

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

    nb::class_<ob::GoalRegion, ob::Goal, PyGoalRegion /* <-- trampoline */>(m, "GoalRegion")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("isSatisfied", nb::overload_cast<const ob::State *>(&ob::GoalRegion::isSatisfied, nb::const_),
             nb::arg("state"))
        .def("isSatisfied", nb::overload_cast<const ob::State *, double *>(&ob::GoalRegion::isSatisfied, nb::const_),
             nb::arg("state"), nb::arg("distance"))
        .def("setThreshold", &ob::GoalRegion::setThreshold, nb::arg("threshold"))
        .def("getThreshold", &ob::GoalRegion::getThreshold)
        .def("print", [](const ob::GoalRegion &self) { self.print(std::cout); })
        .def("__repr__",
             [](const ob::GoalRegion &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             });
}
