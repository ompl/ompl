#include <nanobind/nanobind.h>
#include "ompl/base/goals/GoalState.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalState(nb::module_& m)
{
     // TODO [ob::GoalState][TEST]
    nb::class_<ob::GoalState, ob::GoalSampleableRegion>(m, "GoalState")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"),
             "Construct a GoalState for the given SpaceInformation")
        .def("sampleGoal", &ob::GoalState::sampleGoal, nb::arg("state"),
             "Sample a new goal state into `state`")
        .def("maxSampleCount", &ob::GoalState::maxSampleCount,
             "Return the maximum number of samples that can be generated")
        .def("distanceGoal", &ob::GoalState::distanceGoal, nb::arg("state"),
             "Return the distance from the given state to the goal state")
        .def("print", [](const ob::GoalState &self) { self.print(std::cout); },
             "Print a representation of this GoalState to stdout")
        .def("setState", nb::overload_cast<const ob::State *>(&ob::GoalState::setState), nb::arg("state"),
             "Set the target goal state from a raw State pointer")
        .def("setState", nb::overload_cast<const ob::ScopedState<> &>(&ob::GoalState::setState), nb::arg("state"),
             "Set the target goal state from a ScopedState<>")
        .def("getState", nb::overload_cast<>(&ob::GoalState::getState, nb::const_),
             nb::rv_policy::reference_internal,
             "Get the stored goal state (const)")
        .def("getState", nb::overload_cast<>(&ob::GoalState::getState),
             nb::rv_policy::reference_internal,
             "Get the stored goal state (mutable)");
}
