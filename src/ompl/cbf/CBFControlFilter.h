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
    /// which spheres matter: the solver finds the active ones itself. That removes
    /// both the heuristic active-set preselection and the constraint-refinement
    /// loop that a primal solver needs.
    ///
    /// ### Screening, and what it trades
    ///
    /// Building the forty rows costs about three times the solve, so the rows are
    /// where the money is. `screening` (on by default) uses
    /// `ClearanceBarrier::decreaseRates()` to drop, before computing anything, the
    /// spheres whose clearance provably cannot reach zero within the step — leaving
    /// them at the cost of one interpolated distance each, which is what a plain
    /// collision check would have paid anyway.
    ///
    /// The trade is real and worth stating: the CBF decay condition is then enforced
    /// only for the spheres that survived screening. The others are guaranteed
    /// **safe** by the Lipschitz bound, but not to decay at rate `gamma`. Safety is
    /// the invariant; the decay rate is a smoothness preference. Turn screening off
    /// to recover the exact original semantics, at roughly twice the cost per step.
    ///
    /// ### The certificate, which is the same bound spent differently
    ///
    /// Screening asks "which rows can bind within this step?" and usually answers
    /// "none". The five-argument `filter()` asks the complementary question — "how
    /// long until one could?" — and hands the answer back as
    /// `Diagnostics::certifiedDuration`. Over that span the filter is provably a
    /// no-op, so a caller integrating the returned control across it gets the motion
    /// repeated filtering would have produced, without the calls. In open space that
    /// collapses a whole tree extension into one evaluation and one straight line;
    /// in clutter the certificate is short and the caller steps as it always did.
    ///
    /// It costs one 40x6 matvec on top of an evaluation that has already happened, so
    /// it is always computed. What it is *not* is a licence to take a longer QP step:
    /// the linearization error the margin absorbs still grows with the step, which is
    /// why the certificate is a Lipschitz bound over the interval rather than an
    /// extrapolation of the rows.
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
            /// Skip constraint rows for spheres that cannot bind within the step.
            /// See the class comment for what this gives up.
            bool screening{true};
        };

        /// Optional per-call detail, for diagnostics and benchmarking.
        struct Diagnostics
        {
            double worstValue{0.0};            ///< min_i h_i(q) over the world spheres
            std::size_t worstSphere{0};        ///< which sphere that was
            double worstSelfValue{0.0};        ///< min_p h_ab(q) over the self-collision pairs
            std::size_t worstSelfPair{0};      ///< which pair that was
            bool inBounds{true};               ///< were all centers inside the SDF?
            std::ptrdiff_t solverIterations{0};  ///< qpmad active-set iterations
            int activeRows{ClearanceBarrier::nConstraints};  ///< rows that survived screening
            /// How long the returned control stays certified; see `ControlFilter`'s
            /// five-argument `filter()` and `ClearanceBarrier::certifiedDuration()`.
            double certifiedDuration{0.0};
        };

        /// \p barrier is not copied and must outlive this filter.
        explicit CBFControlFilter(const ClearanceBarrier &barrier);
        CBFControlFilter(const ClearanceBarrier &barrier, const Parameters &parameters);
        ~CBFControlFilter() override;

        Status filter(const Configuration &q, const Control &nominal, double duration,
                      Control &filtered) const override;

        Status filter(const Configuration &q, const Control &nominal, double duration,
                      Control &filtered, double &certified) const override;

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
