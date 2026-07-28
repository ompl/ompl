#pragma once

#include <cstddef>
#include <limits>
#include <memory>

#include <Eigen/Core>

#include <ompl/cbf/ClearanceBarrier.h>
#include <ompl/cbf/ControlFilter.h>

namespace ompl::cbf
{
    /// A control barrier function safety filter, solved as a small QP.
    ///
    /// The nominal control is projected onto the set of controls that keep every
    /// collision sphere's clearance from decaying too fast:
    ///
    ///     minimize    0.5 (u - uNom)^T W (u - uNom)
    ///     subject to  (dh_i/dq) u  >=  -gamma * h_i(q) / dt      for every sphere i
    ///                 uMin <= u <= uMax
    ///
    /// The constraint is the discrete-time CBF condition `h_i(q + u dt) >=
    /// (1 - gamma) h_i(q)` with `h_i(q + u dt)` linearized about `q`. With
    /// `gamma = 1` the clearance is allowed to reach zero in a single step, which
    /// leaves no room for the linearization to be wrong; smaller values keep a
    /// buffer and make the safe set forward invariant rather than merely
    /// non-negative at the next step. The linearization and geometric errors are
    /// absorbed by `ClearanceBarrier`'s margin, not here.
    ///
    /// ### Why the QP is cheap
    ///
    /// Six variables, forty one-sided rows, and a diagonal positive-definite
    /// Hessian. qpmad's dual active set starts from the *unconstrained* minimum,
    /// which for this objective is exactly `uNom`, and activates constraints only
    /// as they turn out to be violated. So a step that is already safe returns
    /// `uNom` untouched at almost no cost, and there is no need to guess in advance
    /// which spheres matter: all forty rows are handed over and the solver finds
    /// the active ones itself. That removes both the heuristic active-set
    /// preselection and the constraint-refinement loop that a primal solver needs.
    ///
    /// ### Threading
    ///
    /// Not thread safe: the solver and its scratch space are reused across calls.
    /// Give each thread its own filter.
    class CBFControlFilter : public ControlFilter
    {
    public:
        struct Parameters
        {
            /// CBF decay rate in (0, 1]: `h(q+) >= (1 - gamma) h(q)`. Smaller is
            /// more conservative. 1.0 permits clearance to reach zero in one step.
            double gamma{0.4};
            /// Diagonal of W: the relative cost of deviating on each joint.
            Control weights{Control::Ones()};
            /// Per-joint speed limit, |u_j| <= maxSpeed_j.
            Control maxSpeed{robots::UR5::velocityLimits()};
            /// Also constrain u so that q + u*dt stays inside the joint limits.
            bool respectJointLimits{true};
        };

        /// Optional per-call detail, for diagnostics and benchmarking.
        struct Diagnostics
        {
            double worstValue{0.0};            ///< min_i h_i(q)
            std::size_t worstSphere{0};        ///< which sphere that was
            bool inBounds{true};               ///< were all centers inside the SDF?
            std::ptrdiff_t solverIterations{0};  ///< qpmad active-set iterations
        };

        /// \p barrier is not copied and must outlive this filter.
        explicit CBFControlFilter(const ClearanceBarrier &barrier);
        CBFControlFilter(const ClearanceBarrier &barrier, const Parameters &parameters);
        ~CBFControlFilter() override;

        Status filter(const Configuration &q, const Control &nominal, double duration,
                      Control &filtered) const override;

        /// As above, additionally reporting why.
        Status filter(const Configuration &q, const Control &nominal, double duration, Control &filtered,
                      Diagnostics &diagnostics) const;

        const char *name() const override
        {
            return "cbf-qp";
        }

        const Parameters &parameters() const
        {
            return parameters_;
        }

        void setParameters(const Parameters &parameters)
        {
            parameters_ = parameters;
        }

        const ClearanceBarrier &barrier() const
        {
            return barrier_;
        }

        /// The control box actually enforced at \p q for a step of \p duration: the
        /// speed limit intersected with what the joint limits allow over that step.
        /// Exposed because a directed control sampler wants to clamp its nominal
        /// control the same way.
        void controlBounds(const Configuration &q, double duration, Control &lower, Control &upper) const;

    private:
        struct Solver;  // hides qpmad from this header

        const ClearanceBarrier &barrier_;
        Parameters parameters_;
        std::unique_ptr<Solver> solver_;
    };
}  // namespace ompl::cbf
