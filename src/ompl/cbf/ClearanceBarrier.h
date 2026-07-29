#pragma once

#include <cstddef>
#include <limits>

#include <Eigen/Core>

#include <ompl/robots/UR5.h>
#include <ompl/sdf/GridSDF.h>

namespace ompl::cbf
{
    /// The barrier function a CBF steering step constrains, for a spherized UR5
    /// in a baked workspace signed distance field.
    ///
    /// For each collision sphere i with radius r_i and center p_i(q):
    ///
    ///     h_i(q) = d(p_i(q)) - r_i - margin
    ///
    /// and, by the chain rule through the sphere-center Jacobian,
    ///
    ///     dh_i/dq = (dp_i/dq)^T grad d(p_i)
    ///
    /// `evaluate()` returns both: the values say how much clearance each sphere
    /// has, and the rows are the linear constraint rows a QP steering step needs.
    /// The robot is safe exactly when every h_i >= 0.
    ///
    /// This class is deliberately tied to `robots::UR5` for now. The only things
    /// it asks of a robot are sphere centers, sphere Jacobians, and radii, so it
    /// generalizes to other spherized robots (or to a mobile base contributing
    /// extra Jacobian columns) behind an interface later.
    ///
    /// ### The margin
    ///
    /// `margin` is what makes the barrier *conservative*, and it has to absorb
    /// three separate errors, none of which are optional:
    ///
    /// - **Sphere under-coverage.** VAMP's sphere model does not enclose the UR5's
    ///   links; mesh vertices sit up to 30.5 mm outside it
    ///   (`scripts/ur5_sphere_coverage.py`). Without this term, h_i > 0 does not
    ///   imply the real robot is clear.
    /// - **SDF discretization.** `GridSDF` interpolates, with error up to about a
    ///   third of a voxel, and it can err optimistically.
    /// - **Step linearization.** A QP using `rows` reasons about a *linear* model
    ///   of h over a finite step; the true h is curved, because the sphere centers
    ///   travel on arcs.
    ///
    /// `defaultMargin` is a single number chosen to cover all three at a 25 mm
    /// voxel and a joint step of ~0.08 rad. It is deliberately blunt — a tighter,
    /// per-sphere bound is derivable from the Jacobian column norms (each one is
    /// the sphere's distance to that joint's axis), but that wants validating
    /// against brute-force rollouts before anything relies on it.
    ///
    /// Note that a *filter* enforcing h >= 0 does not by itself deliver h >= 0: see
    /// `interpolationBuffer()` and `guarding()`, which is what to use when the
    /// filter has replaced the collision checker.
    ///
    /// ### Staying inside the field
    ///
    /// `GridSDF` clamps out-of-bounds queries to the nearest boundary node, which
    /// *over*-reports clearance — the one failure direction a barrier cannot
    /// tolerate. So every evaluation reports whether all sphere centers were
    /// inside the baked box, and a caller must treat `inBounds == false` as "no
    /// usable barrier here" rather than as safety.
    class ClearanceBarrier
    {
    public:
        using Robot = robots::UR5;
        using Configuration = Robot::Configuration;

        static constexpr int nSpheres = static_cast<int>(Robot::nSpheres);
        static constexpr int nJoints = static_cast<int>(Robot::nJoints);

        /// One barrier value per sphere.
        using Values = Eigen::Matrix<double, nSpheres, 1>;
        /// Row i is dh_i/dq — the constraint row sphere i contributes.
        using Rows = Eigen::Matrix<double, nSpheres, nJoints>;

        /// Covers sphere under-coverage (30.5 mm) plus SDF discretization plus
        /// step linearization. See the class comment.
        static constexpr double defaultMargin = 0.06;

        struct Evaluation
        {
            Values values;                ///< h_i(q), always for every sphere
            Rows rows;                    ///< dh_i/dq -- only the first `active` are filled
            /// Which sphere each of the first `active` rows belongs to, so a caller can
            /// pair row r with `values[sphere[r]]`. The identity for a full evaluation.
            Eigen::Matrix<int, nSpheres, 1> sphere;
            int active{nSpheres};         ///< how many rows of `rows` are meaningful
            std::size_t worst{0};         ///< index of the smallest h_i
            bool inBounds{true};          ///< were all centers inside the SDF's box?
        };

        /// How fast each sphere's barrier can possibly fall, per unit time, given a
        /// per-joint speed limit: `rate_i = maxGrad * sum_k maxSpeed_k * L[i][k]`.
        ///
        /// This is the Lipschitz bound that makes screening sound. Over a step of duration
        /// dt, `h_i` cannot decrease by more than `rate_i * dt`, so a sphere with
        /// `h_i > rate_i * dt` **cannot** reach zero during the step whatever control is
        /// applied — its constraint row cannot bind and need not be built, let alone
        /// solved against. Both factors are configuration-independent:
        /// `Robot::leverArmBounds()` bounds `|dp_i/dq_k|` over all of configuration space,
        /// and `GridSDF::maxGradientNorm()` bounds the field's gradient over the whole
        /// grid.
        ///
        /// The bound is conservative twice over — the lever arms are not tight, and it
        /// assumes every joint runs at full speed in the worst direction simultaneously —
        /// so it over-selects rows rather than under-selecting them. That is the safe
        /// direction: a loose bound costs work, a tight-but-wrong one costs safety.
        Values decreaseRates(const Configuration &maxSpeed) const
        {
            return field_.maxGradientNorm() *
                   (Robot::leverArmBounds() * maxSpeed.cwiseAbs()).eval();
        }

        /// Barrier values for every sphere, but constraint rows only for the spheres whose
        /// clearance is at or below \p threshold — normally `decreaseRates(maxSpeed) * dt`.
        ///
        /// The saving is the point: a skipped sphere costs one interpolated *value*, while
        /// an included one costs an interpolated gradient and a Jacobian contraction on
        /// top, and then a row in the QP. In open space almost every sphere is skipped and
        /// this collapses to the cost of a collision check.
        ///
        /// What a caller gives up, and it is a real change rather than an optimisation:
        /// the discrete CBF condition `h(q + u dt) >= (1 - gamma) h(q)` is enforced only
        /// for the spheres that were included. Skipped spheres are guaranteed to stay
        /// **safe** (`h_i > 0`) by the Lipschitz argument above, but not to decay at the
        /// prescribed rate. Safety is the invariant that matters; the decay rate is a
        /// smoothness preference. Audit rather than assume — see `guarding()`.
        void evaluateScreened(const Configuration &q, const Values &threshold, Evaluation &out) const
        {
            const Robot::Kinematics kin = robot_.kinematics(q);

            out.inBounds = true;
            out.worst = 0;
            out.active = 0;
            double smallest = std::numeric_limits<double>::infinity();

            // Centres are kept because the survivors need them again for the gradient
            // query, and forward kinematics is not worth repeating.
            Robot::SphereCenters centers;

            for (std::size_t i = 0; i < Robot::nSpheres; ++i)
            {
                const Eigen::Index index = static_cast<Eigen::Index>(i);
                const Eigen::Vector3d center = Robot::sphereCenter(kin, i);
                centers.col(index) = center;
                out.inBounds = out.inBounds && field_.inBounds(center);

                const double h = field_.distance(center) - Robot::spheres()[i].radius - margin_;
                out.values[index] = h;
                if (h < smallest)
                {
                    smallest = h;
                    out.worst = i;
                }
            }

            for (std::size_t i = 0; i < Robot::nSpheres; ++i)
            {
                const Eigen::Index index = static_cast<Eigen::Index>(i);
                if (out.values[index] > threshold[index])
                    continue;

                const Eigen::Vector3d gradient = field_.gradient(centers.col(index));
                const Eigen::Index row = out.active++;
                out.sphere[row] = static_cast<int>(i);
                out.rows.row(row) = Robot::barrierGradient(kin, i, gradient).transpose();
            }
        }

        /// Neither \p robot nor \p field is copied; both must outlive this object.
        ClearanceBarrier(const Robot &robot, const sdf::GridSDF &field, double margin = defaultMargin)
          : robot_(robot), field_(field), margin_(margin)
        {
        }

        /// How much a *filter* must over-reserve so that the invariant it enforces
        /// still holds when checked afterwards.
        ///
        /// A discrete CBF step certifies h(q + u dt) >= (1-gamma) h(q) using a
        /// linear model built from `rows`. Against an interpolated field that
        /// prediction has a floor on its accuracy: `GridSDF` interpolates values and
        /// node gradients *independently*, so the gradient a row was built from and
        /// the value queried one step later need not agree to better than a fraction
        /// of a voxel. A filter riding the h = 0 boundary therefore dips slightly
        /// below it, by an amount that scales with the grid rather than with the step
        /// — measured at up to 11 mm on a 30 mm grid, i.e. sub-voxel, and *not*
        /// reduced by shrinking gamma or the step size.
        ///
        /// So one voxel. This matters specifically when a CBF filter replaces the
        /// collision checker: with a checker in the loop those steps were quietly
        /// truncated as invalid, and nobody noticed.
        static double interpolationBuffer(const sdf::GridSDF &field)
        {
            return field.spacing().maxCoeff();
        }

        /// The barrier a *filter* should enforce if you intend to hold the robot to
        /// \p margin. Buffered by `interpolationBuffer()`, so auditing the resulting
        /// motion against a plain `ClearanceBarrier(robot, field, margin)` passes.
        ///
        /// Use this whenever the filter is the only thing standing between the
        /// planner and the obstacles.
        static ClearanceBarrier guarding(const Robot &robot, const sdf::GridSDF &field,
                                         double margin = defaultMargin)
        {
            return guarding(robot, field, margin, interpolationBuffer(field));
        }

        /// As above with the buffer chosen by hand. Smaller is cheaper — the buffer
        /// is clearance the planner cannot use, which is what decides whether a tight
        /// passage stays solvable — but a buffer below what the field and step size
        /// actually require silently reintroduces violations. Audit before trusting
        /// any hand-picked value: interpolate a solution path and evaluate the
        /// unbuffered barrier at every step.
        static ClearanceBarrier guarding(const Robot &robot, const sdf::GridSDF &field, double margin,
                                         double buffer)
        {
            return ClearanceBarrier(robot, field, margin + buffer);
        }

        /// Barrier values and constraint rows at \p q.
        void evaluate(const Configuration &q, Evaluation &out) const
        {
            const Robot::Kinematics kin = robot_.kinematics(q);

            out.inBounds = true;
            out.worst = 0;
            out.active = nSpheres;
            double smallest = std::numeric_limits<double>::infinity();

            for (std::size_t i = 0; i < Robot::nSpheres; ++i)
            {
                const Eigen::Vector3d center = Robot::sphereCenter(kin, i);
                out.inBounds = out.inBounds && field_.inBounds(center);
                out.sphere[static_cast<Eigen::Index>(i)] = static_cast<int>(i);

                const sdf::ValueGradient vg = field_.valueAndGradient(center);
                const double h = vg.value - Robot::spheres()[i].radius - margin_;

                const Eigen::Index row = static_cast<Eigen::Index>(i);
                out.values[row] = h;
                out.rows.row(row) = Robot::barrierGradient(kin, i, vg.gradient).transpose();

                if (h < smallest)
                {
                    smallest = h;
                    out.worst = i;
                }
            }
        }

        Evaluation evaluate(const Configuration &q) const
        {
            Evaluation out;
            evaluate(q, out);
            return out;
        }

        /// Barrier values only. Skips the Jacobians, so this is the cheap call for
        /// "is this configuration safe?" — no constraint rows are produced.
        void values(const Configuration &q, Values &out, bool *inBounds = nullptr) const
        {
            const Robot::Kinematics kin = robot_.kinematics(q);
            if (inBounds != nullptr)
                *inBounds = true;

            for (std::size_t i = 0; i < Robot::nSpheres; ++i)
            {
                const Eigen::Vector3d center = Robot::sphereCenter(kin, i);
                if (inBounds != nullptr)
                    *inBounds = *inBounds && field_.inBounds(center);
                out[static_cast<Eigen::Index>(i)] =
                    field_.distance(center) - Robot::spheres()[i].radius - margin_;
            }
        }

        Values values(const Configuration &q) const
        {
            Values out;
            values(q, out);
            return out;
        }

        /// Tightest clearance over all spheres. Safe iff this is >= 0.
        double worstValue(const Configuration &q) const
        {
            return values(q).minCoeff();
        }

        bool isSafe(const Configuration &q) const
        {
            return worstValue(q) >= 0.0;
        }

        double margin() const
        {
            return margin_;
        }

        void setMargin(double margin)
        {
            margin_ = margin;
        }

        const Robot &robot() const
        {
            return robot_;
        }

        const sdf::GridSDF &field() const
        {
            return field_;
        }

    private:
        const Robot &robot_;
        const sdf::GridSDF &field_;
        double margin_;
    };
}  // namespace ompl::cbf
