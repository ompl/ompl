#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/geometric/HillClimbing.h"
#include "ompl/base/goals/GoalRegion.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_HillClimbing(nb::module_ &m)
{
    nb::class_<og::HillClimbing>(m, "HillClimbing")
        .def(nb::init<ob::SpaceInformationPtr>(), nb::arg("si"))
        .def(
            "tryToImprove",
            [](const og::HillClimbing &self, const ob::GoalRegion &goal, ob::State *state, double nearDistance,
               nb::object betterGoalDistance)
            {
                double better = 0.0;
                double *ptr = betterGoalDistance.is_none() ? nullptr : &better;
                bool ok = self.tryToImprove(goal, state, nearDistance, ptr);
                if (ptr != nullptr)
                    return nb::make_tuple(ok, better);
                return nb::make_tuple(ok, nb::none());
            },
            nb::arg("goal"), nb::arg("state"), nb::arg("nearDistance"), nb::arg("betterGoalDistance") = nb::none())
        .def("setMaxImproveSteps", &og::HillClimbing::setMaxImproveSteps, nb::arg("steps"))
        .def("getMaxImproveSteps", &og::HillClimbing::getMaxImproveSteps)
        .def("setValidityCheck", &og::HillClimbing::setValidityCheck, nb::arg("valid"))
        .def("getValidityCheck", &og::HillClimbing::getValidityCheck);
}
