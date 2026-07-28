#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/ControlFilter.h>
#include <ompl/util/Exception.h>

namespace ompl::cbf
{
    /// A joint-space state space whose `interpolate()` is a CBF rollout instead of a
    /// straight line.
    ///
    /// This is the geometric counterpart to `FilteredStatePropagator`, and it exists
    /// because control-space planning turned out to be the wrong shape for speed. A
    /// control edge is capped at `maxControlDuration * stepSize * maxSpeed` of joint
    /// travel, so a wide motion needs many short edges and the tree grows deep:
    /// measured at ~110-770 vertices where `geometric::RRTConnect` needs 5 on the same
    /// scene. Putting the rollout behind `interpolate()` lets a *geometric* planner
    /// take long-range extensions while every intermediate state is still certified by
    /// the barrier.
    ///
    /// ### The contract, and where it bends
    ///
    /// OMPL asks that `interpolate(a, b, 1, out)` give `out == b`. That holds here only
    /// when the rollout is unobstructed — and when it is, this space *is* a
    /// `RealVectorStateSpace`, up to quantisation: the nominal control re-aims at `b`
    /// every step, so in free space the rollout advances exactly `|b - a| / N` per step
    /// and lands on `b`. Near an obstacle the rollout slides along the boundary and
    /// ends somewhere else, which is the entire point: the deflected endpoint is a
    /// perfectly good new tree node, and it is safe, whereas a straight line into the
    /// obstacle is not.
    ///
    /// What this buys, and the reason it composes with stock planners: a planner that
    /// stores `interpolate()`'s output as a node is storing a state the rollout
    /// actually produced. `geometric::RRT` does exactly that on its long-extension
    /// branch (`d > maxDistance_`). On the short branch it stores the raw sample
    /// instead, which the rollout may not reach — that case is what
    /// `FilteredMotionValidator` is for.
    ///
    /// Because the executed motion between two waypoints is by definition
    /// `interpolate()` between them, `PathGeometric::interpolate()` reconstructs the
    /// true trajectory rather than a straight-line fiction. That self-consistency is
    /// why the rollout belongs here and not in the motion validator alone.
    ///
    /// ### What it does not give you
    ///
    /// `distance()` is still Euclidean, and is now a *lower bound* on the length of
    /// the motion `interpolate()` would actually take, since sliding around an
    /// obstacle is longer than cutting through it. That only makes it a slightly
    /// looser nearest-neighbour heuristic, which is all a tree planner needs it for.
    class FilteredStateSpace : public base::RealVectorStateSpace
    {
    public:
        using Configuration = ControlFilter::Configuration;
        using Control = ControlFilter::Control;

        static constexpr int dimension = Configuration::RowsAtCompileTime;

        struct Rollout
        {
            Configuration end;            ///< where the rollout finished
            unsigned int steps{0};        ///< filter calls made
            unsigned int filtered{0};     ///< of those, how many the CBF altered
            unsigned int blocked{0};      ///< of those, how many had no safe control
            bool reachedTarget{false};    ///< did it finish within reachTolerance of `to`?
        };

        /// Aggregate counters, so a planner run can be costed without instrumenting
        /// the planner. Mutable because `interpolate()` is const.
        struct Statistics
        {
            std::size_t rollouts{0};
            std::size_t steps{0};
            std::size_t filtered{0};
            std::size_t blocked{0};
            std::size_t abandoned{0};  ///< rollouts discarded for making no progress
        };

        /// \p filter is not copied and must outlive this space. \p stepSize is the
        /// rollout integration step -- the CBF's linearisation is only valid over a
        /// small step, so this is a correctness parameter, not a performance knob.
        FilteredStateSpace(const ControlFilter &filter, double stepSize, const Control &maxSpeed)
          : base::RealVectorStateSpace(dimension), filter_(filter), stepSize_(stepSize), maxSpeed_(maxSpeed)
        {
            if (stepSize <= 0.0)
                throw Exception("FilteredStateSpace: stepSize must be positive");
            if ((maxSpeed.array() <= 0.0).any())
                throw Exception("FilteredStateSpace: every maxSpeed entry must be positive");
            setName("Filtered" + getName());
        }

        /// How close the rollout must finish to `to` for it to count as having got
        /// there. Exact equality is the right answer in free space but useless near an
        /// obstacle, where two rollouts from the same state toward slightly different
        /// targets deflect slightly differently. Defaults to one full-speed step.
        double reachTolerance() const
        {
            return reachTolerance_ > 0.0 ? reachTolerance_ : maxSpeed_.norm() * stepSize_;
        }

        void setReachTolerance(double tolerance)
        {
            reachTolerance_ = tolerance;
        }

        /// The number of steps a full (t = 1) rollout from \p from to \p to takes: the
        /// time the straight-line motion would need at the per-joint speed limit,
        /// divided by the step size. At least one whenever the states differ.
        unsigned int horizonSteps(const Configuration &from, const Configuration &to) const
        {
            const double horizon = ((to - from).cwiseAbs().cwiseQuotient(maxSpeed_)).maxCoeff();
            return static_cast<unsigned int>(std::ceil(horizon / stepSize_ - 1e-12));
        }

        /// Roll out from \p from toward \p to for \p fraction of the full horizon.
        ///
        /// The nominal control re-aims at \p to at every step rather than being held
        /// constant. Both choices reduce to a straight line in free space, but
        /// re-aiming also recovers the original intent after the filter deflects it,
        /// which is what makes a long extension useful rather than merely safe.
        Rollout roll(const Configuration &from, const Configuration &to, double fraction) const
        {
            Rollout out;
            out.end = from;

            const unsigned int total = horizonSteps(from, to);
            const auto steps = static_cast<unsigned int>(
                std::llround(std::clamp(fraction, 0.0, 1.0) * static_cast<double>(total)));

            Control nominal;
            Control applied;
            for (unsigned int i = 0; i < steps; ++i)
            {
                // Time left in the *full* horizon, so a truncated rollout follows the
                // same trajectory as the prefix of a complete one.
                const double remaining = static_cast<double>(total - i) * stepSize_;
                for (int j = 0; j < dimension; ++j)
                    nominal[j] = std::clamp((to[j] - out.end[j]) / remaining, -maxSpeed_[j], maxSpeed_[j]);

                const ControlFilter::Status status = filter_.filter(out.end, nominal, stepSize_, applied);
                if (status == ControlFilter::Status::Blocked)
                {
                    // Nothing safe to do. Stop rather than sit still burning steps --
                    // the caller gets a short rollout and can decide.
                    ++out.blocked;
                    break;
                }
                if (status == ControlFilter::Status::Filtered)
                    ++out.filtered;

                out.end += applied * stepSize_;
                ++out.steps;
            }

            out.reachedTarget = out.steps == steps && (out.end - to).norm() <= reachTolerance();

            statistics_.rollouts += 1;
            statistics_.steps += out.steps;
            statistics_.filtered += out.filtered;
            statistics_.blocked += out.blocked;
            return out;
        }

        Rollout roll(const base::State *from, const base::State *to, double fraction) const
        {
            return roll(configurationOf(from), configurationOf(to), fraction);
        }

        /// Minimum share of the free-space progress an extension must actually achieve
        /// to be reported at all. See `interpolate()`.
        double minProgressFraction() const
        {
            return minProgressFraction_;
        }

        void setMinProgressFraction(double fraction)
        {
            minProgressFraction_ = fraction;
        }

        void interpolate(const base::State *from, const base::State *to, double t,
                         base::State *state) const override
        {
            // from may alias state, so finish the rollout before writing.
            const Configuration a = configurationOf(from);
            const Configuration b = configurationOf(to);
            const Rollout rollout = roll(a, b, t);

            // An extension that makes no headway toward its target is not an extension,
            // and must be reported as going nowhere rather than as going sideways.
            //
            // This is not a nicety. `RRTConnect::growTree` returns ADVANCED whenever a
            // shortened extension is valid, and its connect loop
            // (`RRTConnect.cpp:294`, `while (gsc == ADVANCED)`) has no
            // termination-condition guard: it assumes each ADVANCED closes the gap by
            // `maxDistance_`, which is true of a straight line and false of a rollout
            // that slides along an obstacle boundary. Without this test that loop spins
            // forever and `solve()` never looks at the clock again -- observed as a hang
            // rather than a timeout.
            //
            // Note the threshold is relative to the progress a *free-space* rollout
            // would have made for this `t`, not absolute, so densifying an accepted edge
            // via `PathGeometric::interpolate()` still behaves.
            const double target = (b - a).norm() * std::clamp(t, 0.0, 1.0);
            const double achieved = (b - a).norm() - (b - rollout.end).norm();
            const bool worthwhile = achieved >= minProgressFraction_ * target;
            if (!worthwhile)
                ++statistics_.abandoned;

            setState(state, worthwhile ? rollout.end : a);
            enforceBounds(state);
        }

        static Configuration configurationOf(const base::State *state)
        {
            const double *values = state->as<StateType>()->values;
            Configuration q;
            for (int j = 0; j < dimension; ++j)
                q[j] = values[j];
            return q;
        }

        static void setState(base::State *state, const Configuration &q)
        {
            for (int j = 0; j < dimension; ++j)
                state->as<StateType>()->values[j] = q[j];
        }

        const ControlFilter &filter() const
        {
            return filter_;
        }

        double stepSize() const
        {
            return stepSize_;
        }

        const Control &maxSpeed() const
        {
            return maxSpeed_;
        }

        const Statistics &statistics() const
        {
            return statistics_;
        }

        void resetStatistics() const
        {
            statistics_ = Statistics();
        }

    private:
        const ControlFilter &filter_;
        double stepSize_;
        Control maxSpeed_;
        double reachTolerance_{-1.0};
        double minProgressFraction_{0.25};
        mutable Statistics statistics_;
    };
}  // namespace ompl::cbf
