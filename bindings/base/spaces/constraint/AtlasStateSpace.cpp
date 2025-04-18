#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_AtlasStateSpace(nb::module_ &m)
{
    //
    // AtlasStateSampler
    //
    nb::class_<ob::AtlasStateSampler, ob::StateSampler>(m, "AtlasStateSampler")
        .def(nb::init<const ob::AtlasStateSpace *>(), nb::arg("space"),
             "Construct an atlas sampler for the given AtlasStateSpace")
        .def("sampleUniform", &ob::AtlasStateSampler::sampleUniform, nb::arg("state"),
             "Sample uniformly on the atlas manifold")
        .def("sampleUniformNear", &ob::AtlasStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"), "Sample uniformly near another state and project onto the manifold")
        .def("sampleGaussian", &ob::AtlasStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"), "Sample from a Gaussian around `mean` and project onto the manifold");

    nb::class_<ompl::base::AtlasStateSpace::StateType, ompl::base::ConstrainedStateSpace::StateType> stateType(m, "AtlasStateType");
    //
    // AtlasStateSpace
    //
    nb::class_<ob::AtlasStateSpace, ob::ConstrainedStateSpace>(m, "AtlasStateSpace")
        .def(nb::init<const ob::StateSpacePtr &, const ob::ConstraintPtr &, bool>(), nb::arg("ambientSpace"),
             nb::arg("constraint"), nb::arg("separate") = true,
             "Construct an AtlasStateSpace from an ambient (unconstrained) space,\n"
             "a Constraint, and an optional `separate` flag")
        .def("clear", &ob::AtlasStateSpace::clear, "Remove all charts and anchors")
        .def("allocDefaultStateSampler", &ob::AtlasStateSpace::allocDefaultStateSampler,
             "Return the default (atlas-aware) sampler")
        .def("allocStateSampler", &ob::AtlasStateSpace::allocStateSampler, "Alias for allocDefaultStateSampler")
        // setters
        .def("setEpsilon", &ob::AtlasStateSpace::setEpsilon, nb::arg("epsilon"), "Set chart radius ε (must be > 0)")
        .def("setRho", &ob::AtlasStateSpace::setRho, nb::arg("rho"), "Set sampling radius Rho (must be > 0)")
        .def("setAlpha", &ob::AtlasStateSpace::setAlpha, nb::arg("alpha"), "Set chart angle Alpha (in (0, π/2))")
        .def("setExploration", &ob::AtlasStateSpace::setExploration, nb::arg("exploration"),
             "Set exploration fraction (in [0,1))")
        .def("setMaxChartsPerExtension", &ob::AtlasStateSpace::setMaxChartsPerExtension, nb::arg("charts"),
             "Set maximum number of new charts per extension")
        .def("setSeparated", &ob::AtlasStateSpace::setSeparated, nb::arg("separate"),
             "Enable/disable sampling in separated charts")
        .def("setBackoff", &ob::AtlasStateSpace::setBackoff, nb::arg("backoff"),
             "Set backoff fraction for chart proposals")
        // getters
        .def("getEpsilon", &ob::AtlasStateSpace::getEpsilon, "Get current ε")
        .def("getRho", &ob::AtlasStateSpace::getRho, "Get current Rho")
        .def("getAlpha", &ob::AtlasStateSpace::getAlpha, "Get current Alpha")
        .def("getExploration", &ob::AtlasStateSpace::getExploration, "Get current exploration fraction")
        .def("getRhoS", &ob::AtlasStateSpace::getRho_s, "Get adjusted sampling radius Rhoₛ")
        .def("getMaxChartsPerExtension", &ob::AtlasStateSpace::getMaxChartsPerExtension,
             "Get charts-per-extension limit")
        .def("isSeparated", &ob::AtlasStateSpace::isSeparated, "Whether sampling is separated per chart")
        .def("getChartCount", &ob::AtlasStateSpace::getChartCount, "Return number of charts currently in the atlas")
        .def("getBackoff", &ob::AtlasStateSpace::getBackoff, "Get chart-backoff fraction")
        // discrete geodesic
        .def(
            "discreteGeodesic",
            [](const ob::AtlasStateSpace &space, const ob::State *from, const ob::State *to, bool interpolate)
            {
                std::vector<ob::State *> geod;
                bool ok = space.discreteGeodesic(from, to, interpolate, &geod);
                return std::make_pair(ok, geod);
            },
            nb::arg("from"), nb::arg("to"), nb::arg("interpolate") = false,
            "Compute a discrete geodesic on the manifold;\n"
            "returns (success: bool, path: List[State])");
}
