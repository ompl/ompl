#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/HybridStateSpace.h"
#include "ompl/base/spaces/HybridTimeStateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_HybridStateSpace(nb::module_ &m)
{
    nb::class_<ob::HybridStateSpace, ob::CompoundStateSpace>(m, "HybridStateSpace")
        .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("spaceComponent"))
        .def("distance", &ob::HybridStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("distanceSpace", &ob::HybridStateSpace::distanceSpace, nb::arg("state1"), nb::arg("state2"))
        .def("distanceTime", &ob::HybridStateSpace::distanceTime, nb::arg("state1"), nb::arg("state2"))
        .def_static("getStateTime", &ob::HybridStateSpace::getStateTime, nb::arg("state"))
        .def_static("getStateJumps", &ob::HybridStateSpace::getStateJumps, nb::arg("state"))
        .def_static("setStateJumps", &ob::HybridStateSpace::setStateJumps, nb::arg("state"), nb::arg("jumps"))
        .def_static("setStateTime", &ob::HybridStateSpace::setStateTime, nb::arg("state"), nb::arg("time"))
        .def(
            "setTimeBounds", [](ob::HybridStateSpace &self, double lower, double upper)
            { self.getTimeComponent()->setTimeBounds(lower, upper); }, nb::arg("lower"), nb::arg("upper"))
        .def("getSpaceComponent", &ob::HybridStateSpace::getSpaceComponent, nb::rv_policy::reference_internal)
        .def("getTimeComponent", &ob::HybridStateSpace::getTimeComponent, nb::rv_policy::reference_internal)
        .def("isMetricSpace", &ob::HybridStateSpace::isMetricSpace)
        .def("getMaximumExtent", &ob::HybridStateSpace::getMaximumExtent);
}
