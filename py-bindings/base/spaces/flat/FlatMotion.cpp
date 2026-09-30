#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <vector>

#include "ompl/base/spaces/flat/FlatMotion.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

using ConstMatrixRef = const Eigen::Ref<const Eigen::MatrixXd> &;

void ompl::binding::base::initSpacesFlat_FlatMotion(nb::module_ &m)
{
    nb::class_<ob::FlatMotion>(m, "FlatMotion")
        .def(nb::init<>())
        .def(nb::init<Eigen::MatrixXd, double>(), nb::arg("coefficients"), nb::arg("duration"))
        .def("coefficients", &ob::FlatMotion::coefficients)
        .def("duration", &ob::FlatMotion::duration)
        .def("degree", &ob::FlatMotion::degree)
        .def("outputDimension", &ob::FlatMotion::outputDimension)
        .def("evaluate", nb::overload_cast<double>(&ob::FlatMotion::evaluate, nb::const_), nb::arg("t"))
        // Returns the derivative rather than writing it into a buffer, since a Python caller gains nothing from
        // reusing one.
        .def(
            "evaluate",
            [](const ob::FlatMotion &motion, double t, unsigned int derivativeLevel)
            {
                Eigen::VectorXd out(motion.outputDimension());
                motion.evaluate(t, derivativeLevel, out);
                return out;
            },
            nb::arg("t"), nb::arg("derivativeLevel"))
        .def("derivative", &ob::FlatMotion::derivative)
        .def("integral", &ob::FlatMotion::integral)
        .def("cost", &ob::FlatMotion::cost, nb::arg("derivativeLevel"), nb::arg("rho"))
        .def("peakSpeed", &ob::FlatMotion::peakSpeed);

    struct PyFlatSteering : ob::FlatSteering
    {
        NB_TRAMPOLINE(ob::FlatSteering, 2);

        std::optional<ob::FlatMotion> steer(ConstMatrixRef from, ConstMatrixRef to) const override
        {
            NB_OVERRIDE_PURE(steer, from, to);
        }

        std::optional<double> steeringCost(ConstMatrixRef from, ConstMatrixRef to) const override
        {
            NB_OVERRIDE(steeringCost, from, to);
        }
    };

    nb::class_<ob::FlatSteering, PyFlatSteering>(m, "FlatSteering")
        .def(nb::init<unsigned int>(), nb::arg("order"))
        .def("getOrder", &ob::FlatSteering::getOrder)
        .def("setRho", &ob::FlatSteering::setRho, nb::arg("rho"))
        .def("getRho", &ob::FlatSteering::getRho)
        .def("cost", &ob::FlatSteering::cost, nb::arg("motion"))
        .def("steer", &ob::FlatSteering::steer, nb::arg("from_"), nb::arg("to"))
        .def("steeringCost", &ob::FlatSteering::steeringCost, nb::arg("from_"), nb::arg("to"));

    nb::class_<ob::FixedDurationSteering, ob::FlatSteering>(m, "FixedDurationSteering")
        .def(nb::init<unsigned int, double>(), nb::arg("order"), nb::arg("duration"))
        .def("setDuration", &ob::FixedDurationSteering::setDuration, nb::arg("duration"))
        .def("getDuration", &ob::FixedDurationSteering::getDuration);

    nb::class_<ob::MinimumEffortSteering, ob::FlatSteering> minimumEffort(m, "MinimumEffortSteering");

    nb::class_<ob::MinimumEffortSteering::OptimalDuration>(minimumEffort, "OptimalDuration")
        .def_ro("duration", &ob::MinimumEffortSteering::OptimalDuration::duration)
        .def_ro("cost", &ob::MinimumEffortSteering::OptimalDuration::cost);

    nb::class_<ob::MinimumEffortSteering::WindingChange>(minimumEffort, "WindingChange")
        .def_ro("time", &ob::MinimumEffortSteering::WindingChange::time)
        .def_ro("winding", &ob::MinimumEffortSteering::WindingChange::winding);

    minimumEffort.def(nb::init<unsigned int>(), nb::arg("order"))
        .def("optimalDuration", &ob::MinimumEffortSteering::optimalDuration, nb::arg("from_"), nb::arg("to"))
        .def("motion", &ob::MinimumEffortSteering::motion, nb::arg("from_"), nb::arg("to"), nb::arg("duration"))
        .def(
            "windingChanges",
            [](const ob::MinimumEffortSteering &steering, ConstMatrixRef from, ConstMatrixRef to, unsigned int axis,
               double period, double horizon)
            {
                const auto changes = steering.windingChanges(from, to, axis, period, horizon);
                return std::vector<ob::MinimumEffortSteering::WindingChange>(changes.begin(), changes.end());
            },
            nb::arg("from_"), nb::arg("to"), nb::arg("axis"), nb::arg("period"), nb::arg("horizon"));
}
