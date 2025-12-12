#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/base/goals/GoalStates.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalStates(nb::module_& m) {
     // TODO [ob::GoalStates][TEST]
    nb::class_<ob::GoalStates, ob::GoalSampleableRegion>(m, "GoalStates")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"),
             "Construct a GoalStates for the given SpaceInformation")
        .def("sampleGoal", &ob::GoalStates::sampleGoal, nb::arg("state"),
             "Sample a new goal state into `state`")
        .def("maxSampleCount", &ob::GoalStates::maxSampleCount,
             "Return the maximum number of samples that can be generated")
        .def("distanceGoal", &ob::GoalStates::distanceGoal, nb::arg("state"),
             "Return the distance from the given state to the closest goal state")
        .def("print", [](const ob::GoalStates &self) {
             self.print(std::cout);
         }, "Return a string representation of this GoalStates")
        .def("addState",
             nb::overload_cast<const ob::State *>(&ob::GoalStates::addState),
             nb::arg("state"),
             "Add a raw State pointer to the set of goal states")
        .def("addState",
             nb::overload_cast<const ob::ScopedState<> &>(&ob::GoalStates::addState),
             nb::arg("state"),
             "Add a ScopedState<> to the set of goal states")
        .def("clear", &ob::GoalStates::clear,
             "Remove all stored goal states")
        .def("hasStates", &ob::GoalStates::hasStates,
             "Return true if any goal states have been added")
        .def("getState",
             &ob::GoalStates::getState,
             nb::arg("index"),
             nb::rv_policy::reference_internal,
             "Return the goal State at the given index (const)")
        .def("getStateCount", &ob::GoalStates::getStateCount,
             "Return the number of stored goal states");
}