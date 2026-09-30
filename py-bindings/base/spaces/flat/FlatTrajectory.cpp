#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/flat/FlatTrajectory.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesFlat_FlatTrajectory(nb::module_ &m)
{
    nb::class_<ob::FlatTrajectory>(m, "FlatTrajectory")
        .def(nb::init<>())
        .def(nb::init<ob::FlatStateSpacePtr, const std::vector<const ob::State *> &>(), nb::arg("space"),
             nb::arg("states"))
        .def(nb::init<const ompl::geometric::PathGeometric &>(), nb::arg("path"))
        .def("size", &ob::FlatTrajectory::size)
        .def("__len__", &ob::FlatTrajectory::size)
        .def("motions", &ob::FlatTrajectory::motions, nb::rv_policy::reference_internal)
        .def("motion", &ob::FlatTrajectory::motion, nb::arg("index"), nb::rv_policy::reference_internal)
        .def("startTime", &ob::FlatTrajectory::startTime, nb::arg("index"))
        .def("duration", &ob::FlatTrajectory::duration)
        .def("outputDimension", &ob::FlatTrajectory::outputDimension)
        .def("evaluate", nb::overload_cast<double>(&ob::FlatTrajectory::evaluate, nb::const_), nb::arg("t"))
        // Returns the derivative rather than writing it into a buffer.
        .def(
            "evaluate",
            [](const ob::FlatTrajectory &trajectory, double t, unsigned int level)
            {
                Eigen::VectorXd out(trajectory.outputDimension());
                trajectory.evaluate(t, level, out);
                return out;
            },
            nb::arg("t"), nb::arg("level"))
        .def("toState", &ob::FlatTrajectory::toState, nb::arg("t"), nb::arg("state"));
}
