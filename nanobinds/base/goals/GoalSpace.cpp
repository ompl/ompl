#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <sstream>
#include "ompl/base/goals/GoalSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initGoals_GoalSpace(nb::module_ &m)
{
    nb::class_<ob::GoalSpace, ob::GoalSampleableRegion>(m, "GoalSpace")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("sampleGoal", &ob::GoalSpace::sampleGoal, nb::arg("state"))
        .def("maxSampleCount", &ob::GoalSpace::maxSampleCount)
        .def("distanceGoal", &ob::GoalSpace::distanceGoal, nb::arg("state"))
        .def("print", [](const ob::GoalSpace &self) { self.print(std::cout); })
        .def("__str__",
             [](const ob::GoalSpace &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
        .def("__repr__",
             [](const ob::GoalSpace &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
        .def("setSpace", &ob::GoalSpace::setSpace, nb::arg("si"))
        .def("getSpace", &ob::GoalSpace::getSpace, nb::rv_policy::reference_internal);
}
