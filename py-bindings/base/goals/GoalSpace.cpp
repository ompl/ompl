#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <sstream>
#include "ompl/base/goals/GoalSpace.h"
#include "ompl/base/StateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    const ob::GoalSpace &withSpace(const ob::GoalSpace &goal)
    {
        if (!goal.getSpace())
            throw nb::value_error("GoalSpace has no space; call setSpace() first");
        return goal;
    }
}  // namespace

void ompl::binding::base::initGoals_GoalSpace(nb::module_ &m)
{
    nb::class_<ob::GoalSpace, ob::GoalSampleableRegion>(m, "GoalSpace")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "sampleGoal", [](const ob::GoalSpace &self, ob::State *state) { withSpace(self).sampleGoal(state); },
            nb::arg("state"))
        .def("maxSampleCount", &ob::GoalSpace::maxSampleCount)
        .def(
            "distanceGoal", [](const ob::GoalSpace &self, const ob::State *state)
            { return withSpace(self).distanceGoal(state); }, nb::arg("state"))
        .def(
            "isSatisfied", [](const ob::GoalSpace &self, const ob::State *state)
            { return withSpace(self).isSatisfied(state); }, nb::arg("state"))
        .def("print", [](const ob::GoalSpace &self) { withSpace(self).print(std::cout); })
        .def("__repr__",
             [](const ob::GoalSpace &self)
             {
                 std::ostringstream oss;
                 withSpace(self).print(oss);
                 return oss.str();
             })
        .def("setSpace", &ob::GoalSpace::setSpace, nb::arg("si"))
        .def("getSpace", &ob::GoalSpace::getSpace);
}
