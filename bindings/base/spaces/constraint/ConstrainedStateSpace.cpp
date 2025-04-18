#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/trampoline.h>

#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/base/Constraint.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

// Trampoline for ConstrainedStateSpace so Python can override its virtuals:
struct PyConstrainedStateSpace : ob::ConstrainedStateSpace {
    // We have 5 virtuals: clear, setup, sanityChecks, discreteGeodesic, geodesicInterpolate
    NB_TRAMPOLINE(ConstrainedStateSpace, 5);

    // Pure virtual: must be overridden in Python
    bool discreteGeodesic(const ob::State *from,
                          const ob::State *to,
                          bool interpolate,
                          std::vector<ob::State *> *geodesic) const override
    {
        NB_OVERRIDE_PURE(discreteGeodesic, from, to, interpolate, geodesic);
    }

    // Optional override
    ob::State *geodesicInterpolate(const std::vector<ob::State *> &geodesic,
                                double t) const override
    {
        NB_OVERRIDE(geodesicInterpolate, geodesic, t);
    }

    // Optional overrides
    void setup() override { NB_OVERRIDE(setup); }
    void clear() override { NB_OVERRIDE(clear); }
    void sanityChecks() const override { NB_OVERRIDE(sanityChecks); }
};

void ompl::binding::base::initSpacesConstraint_ConstrainedStateSpace(nb::module_ &m) {
    //
    // ConstrainedMotionValidator
    //
    nb::class_<ob::ConstrainedMotionValidator, ob::MotionValidator>(m, "ConstrainedMotionValidator")
        .def(nb::init<ob::SpaceInformation*>(),      nb::arg("si"),
             "Construct from a raw SpaceInformation pointer")
        .def(nb::init<const ob::SpaceInformationPtr&>(), nb::arg("siPtr"),
             "Construct from a shared SpaceInformationPtr")
        .def("checkMotion",
             nb::overload_cast<const ob::State*, const ob::State*>(
                 &ob::ConstrainedMotionValidator::checkMotion, nb::const_),
             nb::arg("s1"), nb::arg("s2"),
             "Check whether motion from s1 to s2 satisfies the constraint");
    
    nb::class_<ompl::base::ConstrainedStateSpace::StateType, ompl::base::WrapperStateSpace::StateType> stateType(m. "ConstrainedStateType");

    //
    // ConstrainedStateSpace
    //
    nb::class_<ob::ConstrainedStateSpace,
               ob::WrapperStateSpace,
               PyConstrainedStateSpace /* <-- trampoline */>(m, "ConstrainedStateSpace")
        .def(nb::init<const ob::StateSpacePtr&, const ob::ConstraintPtr&>(),
             nb::arg("ambientSpace"), nb::arg("constraint"),
             "Wrap an ambient StateSpace with a Constraint; manifold dim and sampling follow the constraint")
        .def("setSpaceInformation",
             &ob::ConstrainedStateSpace::setSpaceInformation,
             nb::arg("si"),
             "Associate a SpaceInformation (with this very ConstrainedStateSpace) for collision etc.")
        .def("setup",
             &ob::ConstrainedStateSpace::setup,
             "Finalize internal data structures after setting up constraint and sampler")
        .def("clear",
             &ob::ConstrainedStateSpace::clear,
             "Reset any cached geodesics or samplers")
        .def("allocState",
             &ob::ConstrainedStateSpace::allocState,
             "Allocate a new constrained state")
        .def("constrainedSanityChecks",
             &ob::ConstrainedStateSpace::constrainedSanityChecks,
             nb::arg("flags"),
             "Run only the constraint-specific sanity checks (pass bitmask of SanityChecks)")
        .def("sanityChecks",
             &ob::ConstrainedStateSpace::sanityChecks,
             "Run all sanity checks, including constraint and ambient space")
        .def("validSegmentCount",
             &ob::ConstrainedStateSpace::validSegmentCount,
             nb::arg("s1"), nb::arg("s2"),
             "How many discrete steps we would take between two states")
        .def("interpolate",
             &ob::ConstrainedStateSpace::interpolate,
             nb::arg("from"), nb::arg("to"), nb::arg("t"), nb::arg("state"),
             "Interpolate along the constrained manifold")
        .def("geodesicInterpolate",
             &ob::ConstrainedStateSpace::geodesicInterpolate,
             nb::arg("geodesic"), nb::arg("t"),
             "Re-interpolate a previously computed geodesic at fractional parameter t")
        .def("setDelta",
             &ob::ConstrainedStateSpace::setDelta,
             nb::arg("delta"),
             "Set the step size Δ for discrete geodesic sampling")
        .def("getDelta",
             &ob::ConstrainedStateSpace::getDelta,
             "Return the current Δ")
        .def("setLambda",
             &ob::ConstrainedStateSpace::setLambda,
             nb::arg("lambda"),
             "Set the overshoot factor λ (>1)")
        .def("getLambda",
             &ob::ConstrainedStateSpace::getLambda,
             "Return the current λ")
        .def("getAmbientDimension",
             &ob::ConstrainedStateSpace::getAmbientDimension,
             "Dimension of the ambient (unconstrained) space")
        .def("getManifoldDimension",
             &ob::ConstrainedStateSpace::getManifoldDimension,
             "Dimension of the constrained manifold")
        .def("getConstraint",
             &ob::ConstrainedStateSpace::getConstraint,
             nb::rv_policy::reference_internal,
             "Return the Constraint object defining this manifold");
}
