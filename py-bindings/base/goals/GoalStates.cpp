#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <memory>
#include <sstream>

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/SpaceInformation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalStates(nb::module_ &m)
{
    nb::class_<ob::GoalStates, ob::GoalSampleableRegion>(m, "GoalStates")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"),
             "Construct a GoalStates for the given SpaceInformation")
        .def("sampleGoal", &ob::GoalStates::sampleGoal, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalStates::maxSampleCount)
        .def("distanceGoal", &ob::GoalStates::distanceGoal, nb::arg("state"))
        .def("print", [](const ob::GoalStates &self) { self.print(std::cout); })
        .def("__repr__",
             [](const ob::GoalStates &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
        .def("addState", nb::overload_cast<const ob::State *>(&ob::GoalStates::addState), nb::arg("state"))
        .def("clear", &ob::GoalStates::clear)
        .def("hasStates", &ob::GoalStates::hasStates)
        // A copy: clear() frees the stored states, which would leave a view dangling.
        .def(
            "getState",
            [](const ob::GoalStates &goal, unsigned int index) -> std::shared_ptr<ob::State>
            {
                const ob::SpaceInformationPtr &si = goal.getSpaceInformation();
                return {si->cloneState(goal.getState(index)),
                        [space = si->getStateSpace()](ob::State *s) { space->freeState(s); }};
            },
            nb::arg("index"))
        .def("getStateCount", &ob::GoalStates::getStateCount);
}
