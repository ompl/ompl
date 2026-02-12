#include <nanobind/nanobind.h>
#include "ompl/base/goals/GoalState.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalState(nb::module_ &m)
{
    nb::class_<ob::GoalState, ob::GoalSampleableRegion>(m, "GoalState")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("sampleGoal", &ob::GoalState::sampleGoal, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalState::maxSampleCount)
        .def("distanceGoal", &ob::GoalState::distanceGoal, nb::arg("state"))
        .def("print", [](const ob::GoalState &self) { self.print(std::cout); })
        .def("__repr__",
             [](const ob::GoalState &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
        .def("setState", nb::overload_cast<const ob::State *>(&ob::GoalState::setState), nb::arg("state"))
        .def("getState", nb::overload_cast<>(&ob::GoalState::getState, nb::const_), nb::rv_policy::reference_internal)
        .def("getState", nb::overload_cast<>(&ob::GoalState::getState));
}
