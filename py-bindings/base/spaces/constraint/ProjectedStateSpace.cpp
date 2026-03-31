#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/constraint/ProjectedStateSpace.h"
#include "ompl/base/Constraint.h"
#include "../../init.h"

namespace nb  = nanobind;
namespace ob  = ompl::base;

void ompl::binding::base::initSpacesConstraint_ProjectedStateSpace(nb::module_ &m) {
    
     // TODO [ob::ProjectedStateSampler][TEST]
     nb::class_<ob::ProjectedStateSampler,
                    ob::WrapperStateSampler>(m, "ProjectedStateSampler")
          .def(nb::init<const ob::ProjectedStateSpace*, ob::StateSamplerPtr>(),
               nb::arg("space"), nb::arg("sampler"))
          .def("sampleUniform",
               &ob::ProjectedStateSampler::sampleUniform,
               nb::arg("state"))
          .def("sampleUniformNear",
               &ob::ProjectedStateSampler::sampleUniformNear,
               nb::arg("state"), nb::arg("near"), nb::arg("distance"))
          .def("sampleGaussian",
               &ob::ProjectedStateSampler::sampleGaussian,
               nb::arg("state"), nb::arg("mean"), nb::arg("stdDev"));

     // TODO [ob::ProjectedStateSpace][TEST]
     nb::class_<ob::ProjectedStateSpace,
                    ob::ConstrainedStateSpace>(m, "ProjectedStateSpace")
          .def(nb::init<const ob::StateSpacePtr&, const ob::ConstraintPtr&>(),
               nb::arg("ambientSpace"), nb::arg("constraint"))
          .def("allocDefaultStateSampler",
               &ob::ProjectedStateSpace::allocDefaultStateSampler)
          .def("allocStateSampler",
               &ob::ProjectedStateSpace::allocStateSampler)
          .def("discreteGeodesic",
               [](const ob::ProjectedStateSpace &pss,
                    const ob::State *from,
                    const ob::State *to,
                    bool interpolate) {
                    std::vector<ob::State*> geod;
                    bool ok = pss.discreteGeodesic(from, to, interpolate, &geod);
                    return std::make_pair(ok, geod);
               },
               nb::arg("from"), nb::arg("to"), nb::arg("interpolate") = false);
}
