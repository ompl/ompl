#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/base/goals/GoalStates.h"
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
        .def("__str__",
             [](const ob::GoalStates &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
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
        .def("getState", &ob::GoalStates::getState, nb::arg("index"), nb::rv_policy::reference_internal)
        .def("getStateCount", &ob::GoalStates::getStateCount);
}