#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/SpaceTimeStateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_SpaceTimeStateSpace(nb::module_ &m)
{
    nb::class_<ob::SpaceTimeStateSpace, ob::CompoundStateSpace>(m, "SpaceTimeStateSpace")
        .def(nb::init<const ob::StateSpacePtr &, double, double>(),
             nb::arg("spaceComponent"), nb::arg("vMax") = 1.0, nb::arg("timeWeight") = 0.5)

        .def("distance", &ob::SpaceTimeStateSpace::distance, nb::arg("state1"), nb::arg("state2"))

        .def("timeToCoverDistance", &ob::SpaceTimeStateSpace::timeToCoverDistance,
             nb::arg("state1"), nb::arg("state2"))

        .def("distanceSpace", &ob::SpaceTimeStateSpace::distanceSpace,
             nb::arg("state1"), nb::arg("state2"))

        .def("distanceTime", &ob::SpaceTimeStateSpace::distanceTime,
             nb::arg("state1"), nb::arg("state2"))

        .def_static("getStateTime", &ob::SpaceTimeStateSpace::getStateTime,
                   nb::arg("state"))

        .def("setTimeBounds", &ob::SpaceTimeStateSpace::setTimeBounds,
             nb::arg("lower"), nb::arg("upper"))

        .def("getVMax", &ob::SpaceTimeStateSpace::getVMax)

        .def("setVMax", &ob::SpaceTimeStateSpace::setVMax,
             nb::arg("vMax"))

        .def("getSpaceComponent", &ob::SpaceTimeStateSpace::getSpaceComponent,
             nb::rv_policy::reference_internal)

        .def("getTimeComponent", &ob::SpaceTimeStateSpace::getTimeComponent,
             nb::rv_policy::reference_internal)

        .def("isMetricSpace", &ob::SpaceTimeStateSpace::isMetricSpace)

        .def("getMaximumExtent", &ob::SpaceTimeStateSpace::getMaximumExtent)

        .def("updateEpsilon", &ob::SpaceTimeStateSpace::updateEpsilon);
}
