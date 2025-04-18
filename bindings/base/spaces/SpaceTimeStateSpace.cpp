#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/SpaceTimeStateSpace.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_SpaceTimeStateSpace(nb::module_ &m)
{
    nb::class_<ob::SpaceTimeStateSpace, ob::CompoundStateSpace>(m, "SpaceTimeStateSpace")
        .def(nb::init<const ob::StateSpacePtr &, double, double>(),
             nb::arg("spaceComponent"), nb::arg("vMax") = 1.0, nb::arg("timeWeight") = 0.5,
             "Construct a SpaceTimeStateSpace from a spatial component, a maximum velocity, and a time weight.")

        .def("distance", &ob::SpaceTimeStateSpace::distance, nb::arg("state1"), nb::arg("state2"),
             "Compute the distance between two states in the space-time space.")

        .def("timeToCoverDistance", &ob::SpaceTimeStateSpace::timeToCoverDistance,
             nb::arg("state1"), nb::arg("state2"),
             "Estimate the time required to travel between two states given the maximum velocity.")

        .def("distanceSpace", &ob::SpaceTimeStateSpace::distanceSpace,
             nb::arg("state1"), nb::arg("state2"),
             "Compute the spatial distance component between two states.")

        .def("distanceTime", &ob::SpaceTimeStateSpace::distanceTime,
             nb::arg("state1"), nb::arg("state2"),
             "Compute the temporal distance component between two states.")

        .def_static("getStateTime", &ob::SpaceTimeStateSpace::getStateTime,
                   nb::arg("state"),
                   "Return the time value from the space-time state.")

        .def("setTimeBounds", &ob::SpaceTimeStateSpace::setTimeBounds,
             nb::arg("lower"), nb::arg("upper"),
             "Set the lower and upper bounds for time in the space-time state.")

        .def("getVMax", &ob::SpaceTimeStateSpace::getVMax,
             "Get the maximum allowed velocity.")

        .def("setVMax", &ob::SpaceTimeStateSpace::setVMax,
             nb::arg("vMax"),
             "Set the maximum allowed velocity.")

        .def("getSpaceComponent", &ob::SpaceTimeStateSpace::getSpaceComponent,
             nb::rv_policy::reference_internal,
             "Return the spatial component of the compound state space.")

        .def("getTimeComponent", &ob::SpaceTimeStateSpace::getTimeComponent,
             nb::rv_policy::reference_internal,
             "Return the time component of the compound state space.")

        .def("isMetricSpace", &ob::SpaceTimeStateSpace::isMetricSpace,
             "Return whether this state space is a metric space.")

        .def("getMaximumExtent", &ob::SpaceTimeStateSpace::getMaximumExtent,
             "Return the maximum extent of this state space.")

        .def("updateEpsilon", &ob::SpaceTimeStateSpace::updateEpsilon,
             "Update the epsilon value used for internal comparisons.");
}
