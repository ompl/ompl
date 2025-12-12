#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <sstream>
#include "ompl/base/goals/GoalSpace.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalSpace(nb::module_ &m)
{
     // TODO [ob::GoalSpace][TEST]
    nb::class_<ob::GoalSpace, ob::GoalSampleableRegion>(m, "GoalSpace")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"),
             "Construct a GoalSpace for the given SpaceInformation")
        .def("sampleGoal", &ob::GoalSpace::sampleGoal, nb::arg("state"), "Sample a new goal state into `state`")
        .def("maxSampleCount", &ob::GoalSpace::maxSampleCount,
             "Return the maximum number of samples that can be generated.")
        .def("distanceGoal", &ob::GoalSpace::distanceGoal, nb::arg("state"),
             "Return the distance from the given state to the goal region.")
        .def(
            "print", [](const ob::GoalSpace &self) { self.print(std::cout); },
            "Return a string representation of this GoalSpace")
        .def("setSpace", &ob::GoalSpace::setSpace, nb::arg("si"), "Set the Space for this goal space.")
        .def("getSpace", &ob::GoalSpace::getSpace, nb::rv_policy::reference_internal,
             "Return the Space associated with this goal space.");
}
