#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <memory>

#include "ompl/base/Planner.h"
#include "ompl/base/spaces/flat/FlatStateSpace.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesFlat_FlatStateSpace(nb::module_ &m)
{
    nb::class_<ob::FlatStateSpace, ob::CompoundStateSpace> space(m, "FlatStateSpace");

    nb::class_<ob::FlatStateSpace::StateType, ob::CompoundState>(m, "FlatStateType")
        .def("output", nb::overload_cast<>(&ob::FlatStateSpace::StateType::output), nb::rv_policy::reference_internal)
        .def("derivative", nb::overload_cast<unsigned int>(&ob::FlatStateSpace::StateType::derivative),
             nb::arg("level"), nb::rv_policy::reference_internal);

    nb::enum_<ob::FlatStateSpace::DistanceType>(space, "DistanceType")
        .value("FLAT_STATE_METRIC", ob::FlatStateSpace::FLAT_STATE_METRIC)
        .value("TRAJECTORY_COST", ob::FlatStateSpace::TRAJECTORY_COST)
        .export_values();

    space
        .def(nb::init<const ob::StateSpacePtr &, unsigned int, ob::FlatChartPtr>(), nb::arg("output"), nb::arg("order"),
             nb::arg("chart") = nullptr)
        .def(nb::init<const ob::StateSpacePtr &, const std::vector<ob::StateSpacePtr> &, ob::FlatChartPtr>(),
             nb::arg("output"), nb::arg("derivatives"), nb::arg("chart") = nullptr)
        .def("getOrder", &ob::FlatStateSpace::getOrder)
        .def("getOutputDimension", &ob::FlatStateSpace::getOutputDimension)
        .def("getOutputSpace", &ob::FlatStateSpace::getOutputSpace)
        .def("getChart", &ob::FlatStateSpace::getChart)
        // Returns the shared pointer to make usage more practical.
        .def(
            "getDerivativeSpace",
            [](const ob::FlatStateSpace &self, unsigned int level)
            {
                if (level < 1 || level >= self.getOrder())
                    throw nb::index_error("getDerivativeSpace: level must lie between 1 and the order minus one");
                return std::static_pointer_cast<ob::RealVectorStateSpace>(self.getSubspace(level));
            },
            nb::arg("level"))
        .def("setDerivativeBound", &ob::FlatStateSpace::setDerivativeBound, nb::arg("level"), nb::arg("bound"))
        .def("setRho", &ob::FlatStateSpace::setRho, nb::arg("rho"))
        .def("getRho", &ob::FlatStateSpace::getRho)
        .def("getSteering", &ob::FlatStateSpace::getSteering, nb::rv_policy::reference_internal)
        .def("setDistanceType", &ob::FlatStateSpace::setDistanceType, nb::arg("type"))
        .def("getDistanceType", &ob::FlatStateSpace::getDistanceType)
        .def("steeringCost", &ob::FlatStateSpace::steeringCost, nb::arg("from_"), nb::arg("to"))
        .def("steer", &ob::FlatStateSpace::steer, nb::arg("from_"), nb::arg("to"))
        // Returns the flat state rather than writing it into a buffer.
        .def(
            "toFlatState",
            [](const ob::FlatStateSpace &self, const ob::State *state)
            {
                Eigen::MatrixXd flatState(self.getOrder(), self.getOutputDimension());
                self.toFlatState(state, flatState);
                return flatState;
            },
            nb::arg("state"))
        .def("fromFlatState", &ob::FlatStateSpace::fromFlatState, nb::arg("flatState"), nb::arg("state"))
        .def("interpolate",
             nb::overload_cast<const ob::State *, const ob::State *, double, ob::State *>(
                 &ob::FlatStateSpace::interpolate, nb::const_),
             nb::arg("from_"), nb::arg("to"), nb::arg("t"), nb::arg("state"))
        .def("interpolate",
             nb::overload_cast<const ob::State *, const ob::FlatMotion &, double, ob::State *>(
                 &ob::FlatStateSpace::interpolate, nb::const_),
             nb::arg("from_"), nb::arg("motion"), nb::arg("t"), nb::arg("state"))
        .def(
            "validSegmentCount",
            nb::overload_cast<const ob::State *, const ob::State *>(&ob::FlatStateSpace::validSegmentCount, nb::const_),
            nb::arg("state1"), nb::arg("state2"))
        .def("validSegmentCount",
             nb::overload_cast<const ob::FlatMotion &>(&ob::FlatStateSpace::validSegmentCount, nb::const_),
             nb::arg("motion"))
        .def("distance", &ob::FlatStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
        .def("isMetricSpace", &ob::FlatStateSpace::isMetricSpace)
        .def("hasSymmetricDistance", &ob::FlatStateSpace::hasSymmetricDistance)
        .def("hasSymmetricInterpolate", &ob::FlatStateSpace::hasSymmetricInterpolate)
        .def("sanityChecks", nb::overload_cast<>(&ob::FlatStateSpace::sanityChecks, nb::const_))
        .def("checkPlanner", &ob::FlatStateSpace::checkPlanner, nb::arg("planner"))
        .def("registerProjections", &ob::FlatStateSpace::registerProjections)
        .def("setup", &ob::FlatStateSpace::setup)
        .def("allocState", &ob::FlatStateSpace::allocState);
}
