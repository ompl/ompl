#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/constraint/ProjectedStateSpace.h"
#include "ompl/base/Constraint.h"
#include "../../init.hh"

namespace nb  = nanobind;
namespace ob  = ompl::base;

void ompl::binding::base::initSpacesConstraint_ProjectedStateSpace(nb::module_ &m) {
    //
    // ProjectedStateSampler
    //
    nb::class_<ob::ProjectedStateSampler,
               ob::WrapperStateSampler>(m, "ProjectedStateSampler")
        .def(nb::init<const ob::ProjectedStateSpace*, ob::StateSamplerPtr>(),
             nb::arg("space"), nb::arg("sampler"),
             "Wrap an existing sampler so that every sample is projected onto the constraint manifold.")
        .def("sampleUniform",
             &ob::ProjectedStateSampler::sampleUniform,
             nb::arg("state"),
             "Sample uniformly and project the result into `state`.")
        .def("sampleUniformNear",
             &ob::ProjectedStateSampler::sampleUniformNear,
             nb::arg("state"), nb::arg("near"), nb::arg("distance"),
             "Sample uniformly near another state and project.")
        .def("sampleGaussian",
             &ob::ProjectedStateSampler::sampleGaussian,
             nb::arg("state"), nb::arg("mean"), nb::arg("stdDev"),
             "Sample from a Gaussian around `mean` and project.");

    //
    // ProjectedStateSpace
    //
    nb::class_<ob::ProjectedStateSpace,
               ob::ConstrainedStateSpace>(m, "ProjectedStateSpace")
        .def(nb::init<const ob::StateSpacePtr&, const ob::ConstraintPtr&>(),
             nb::arg("ambientSpace"), nb::arg("constraint"),
             "Construct from an ambient (unconstrained) space and a Constraint.")
        .def("allocDefaultStateSampler",
             &ob::ProjectedStateSpace::allocDefaultStateSampler,
             "Return a sampler that projects each draw onto the manifold.")
        .def("allocStateSampler",
             &ob::ProjectedStateSpace::allocStateSampler,
             "Alias for `allocDefaultStateSampler`.")
        .def("discreteGeodesic",
             [](const ob::ProjectedStateSpace &pss,
                const ob::State *from,
                const ob::State *to,
                bool interpolate) {
                 std::vector<ob::State*> geod;
                 bool ok = pss.discreteGeodesic(from, to, interpolate, &geod);
                 return std::make_pair(ok, geod);
             },
             nb::arg("from"), nb::arg("to"), nb::arg("interpolate") = false,
             "Compute a discrete geodesic on the manifold. Returns (success, list_of_states).");
}
