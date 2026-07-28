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
            std::size_t steps{0};  ///< filter calls actually made
            std::size_t filtered{0};
            std::size_t blocked{0};
            std::size_t abandoned{0};  ///< rollouts discarded for making no progress
            std::size_t resumed{0};    ///< rollouts that continued a memoized prefix
            std::size_t stepsSaved{0}; ///< filter calls those prefixes stood in for
            std::size_t certified{0};  ///< motions answered by `certifiedByLastRollout()`
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
        ///
        /// The rollout state at step i is a function of (`from`, `to`, the full horizon,
        /// i) alone — that is what the `remaining` term below buys — so a longer rollout
        /// over the same motion *contains* a shorter one. This exploits that: the last
        /// rollout is memoized and a call that only carries it further resumes from where
        /// it stopped. Without that, `base::SpaceInformation::getMotionStates` (and
        /// therefore `geometric::PathGeometric::interpolate()` and the planners'
        /// `addIntermediateStates` mode) costs O(n^2) filter calls to produce n states,
        /// because it asks for fractions 1/n … n/n and each one restarted from `from`.
        ///
        /// The memo is single-entry and returns identical results to a cold call, so it
        /// is invisible apart from `statistics()` and the loss of thread safety —
        /// which `ControlFilter` already costs anyway, since `CBFControlFilter` solves
        /// into shared scratch.
        Rollout roll(const Configuration &from, const Configuration &to, double fraction) const
        {
            const unsigned int total = horizonSteps(from, to);
            const auto steps = static_cast<unsigned int>(
                std::llround(std::clamp(fraction, 0.0, 1.0) * static_cast<double>(total)));

            Rollout out;
            out.end = from;

            // Resume rather than restart when this is the same motion carried further.
            const bool resumed = memo_.valid && memo_.steps <= steps && memo_.horizon == total &&
                                 sameConfiguration(memo_.from, from) && sameConfiguration(memo_.to, to);
            if (resumed)
            {
                out.end = memo_.end;
                out.steps = memo_.steps;
                out.filtered = memo_.filtered;
                statistics_.resumed += 1;
                statistics_.stepsSaved += memo_.steps;
            }
            const unsigned int filteredBefore = out.filtered;

            // A rollout that stopped early stops in the same place next time: the block
            // happened at step index `out.steps` of this same horizon, so the filter
            // would be handed the identical state and nominal control again.
            bool terminal = resumed && memo_.terminal;

            Control nominal;
            Control applied;
            for (unsigned int i = out.steps; i < steps && !terminal; ++i)
            {
                // Time left in the *full* horizon, so a truncated rollout follows the
                // same trajectory as the prefix of a complete one.
                const double remaining = static_cast<double>(total - i) * stepSize_;
                for (int j = 0; j < dimension; ++j)
                    nominal[j] = std::clamp((to[j] - out.end[j]) / remaining, -maxSpeed_[j], maxSpeed_[j]);

                const ControlFilter::Status status = filter_.filter(out.end, nominal, stepSize_, applied);
                ++statistics_.steps;
                if (status == ControlFilter::Status::Blocked)
                {
                    // Nothing safe to do. Stop rather than sit still burning steps --
                    // the caller gets a short rollout and can decide.
                    terminal = true;
                    break;
                }
                if (status == ControlFilter::Status::Filtered)
                    ++out.filtered;

                out.end += applied * stepSize_;
                ++out.steps;
            }

            // Only a rollout that wanted to go further than the block counts as blocked;
            // one asked for exactly the safe prefix got everything it asked for.
            if (terminal && out.steps < steps)
                out.blocked = 1;
            out.reachedTarget = out.steps == steps && (out.end - to).norm() <= reachTolerance();

            const bool newlyBlocked = out.blocked != 0 && !(resumed && memo_.terminal);
            memo_ = Memo{from, to, out.end, total, out.steps, out.filtered, terminal, true};

            statistics_.rollouts += 1;
            statistics_.filtered += out.filtered - filteredBefore;
            statistics_.blocked += newlyBlocked ? 1 : 0;
            return out;
        }

        Rollout roll(const base::State *from, const base::State *to, double fraction) const
        {
            return roll(configurationOf(from), configurationOf(to), fraction);
        }

        /// True when the memoized rollout is *itself* the proof that a motion from
        /// \p from to \p to is safe and arrives, so there is nothing to roll.
        ///
        /// This exists because a tree planner rolls twice per extension and only needs
        /// to roll once. `geometric::RRT::solve` interpolates toward a random sample
        /// (`RRT.cpp:144`) and then calls `checkMotion` on the state that produced
        /// (`RRT.cpp:148`); `RRTConnect.cpp:130`/`:142` does the same. The second
        /// rollout re-derives a motion the first one just demonstrated.
        ///
        /// The condition is deliberately narrow. It requires the memoized rollout to
        /// have started at \p from, ended exactly at \p to, taken at least one step, and
        /// — the load-bearing part — to have been *unfiltered*. An unfiltered rollout is
        /// the straight line from `from` to its endpoint, so re-rolling with \p to as the
        /// target retraces the same line through the same free space and arrives; the
        /// answer is identical and only the work differs.
        ///
        /// Widening it to filtered rollouts would roughly double the saving and must not
        /// be done. Such an extension is still *safe* — the filter certified every state
        /// it produced — but it would not be **reproducible**: aiming at the deflected
        /// endpoint is a different motion from aiming at the original sample, so a
        /// re-derivation that no longer has the memo can land short. That re-derivation
        /// is exactly what `geometric::PathGeometric::check()` does, and what the demo's
        /// audit relies on, so certifying by construction here would buy speed by
        /// accepting edges the audit then rejects. Filtered rollouts get checked for
        /// real, which is the case worth checking anyway.
        bool certifiedByLastRollout(const Configuration &from, const Configuration &to) const
        {
            const bool certified = memo_.valid && memo_.steps > 0 && memo_.filtered == 0 &&
                                   !memo_.terminal && sameConfiguration(memo_.from, from) &&
                                   sameConfiguration(memo_.end, to);
            if (certified)
                ++statistics_.certified;
            return certified;
        }

        bool certifiedByLastRollout(const base::State *from, const base::State *to) const
        {
            return certifiedByLastRollout(configurationOf(from), configurationOf(to));
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
        /// The last rollout, kept so the next one need not repeat it. See `roll()` and
        /// `certifiedByLastRollout()` for the two ways that pays.
        struct Memo
        {
            Configuration from{Configuration::Zero()};
            Configuration to{Configuration::Zero()};
            Configuration end{Configuration::Zero()};
            unsigned int horizon{0};   ///< `horizonSteps(from, to)` when it was taken
            unsigned int steps{0};
            unsigned int filtered{0};
            bool terminal{false};      ///< stopped on a `Blocked` step, so it will again
            bool valid{false};
        };

        /// Bitwise equality, because the point is to recognise a state the planner just
        /// handed back unmodified. A near-miss is a cache miss, which only costs work.
        static bool sameConfiguration(const Configuration &a, const Configuration &b)
        {
            return (a.array() == b.array()).all();
        }

        const ControlFilter &filter_;
        double stepSize_;
        Control maxSpeed_;
        double reachTolerance_{-1.0};
        double minProgressFraction_{0.25};
        mutable Statistics statistics_;
        mutable Memo memo_;
    };
}  // namespace ompl::cbf
