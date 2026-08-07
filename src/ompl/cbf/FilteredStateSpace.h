#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/ControlFilter.h>
#include <ompl/util/Exception.h>

namespace ompl::cbf
{
    /// A joint-space state space whose `interpolate()` is a CBF rollout instead of a
    /// straight line, and which keeps the trajectory it produced.
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
    /// ### The edge is the trajectory
    ///
    /// A tree edge in stock OMPL is two endpoints; the motion between them is whatever
    /// the state space says it is, recomputed on demand. That does not work here. The
    /// nominal control re-aims at its target every step, so a rollout aimed at the
    /// deflected endpoint `x` is a *different motion* from the one that produced `x`
    /// while aiming at the original sample. Recomputing gives a plausible, safe, wrong
    /// answer -- one that passes every barrier check while not being the trajectory the
    /// planner actually found.
    ///
    /// So the rollout is run once and its waypoints are kept, keyed by the edge they
    /// produced. Every later question about that edge -- is it valid, what does it look
    /// like at fraction t, what should the robot execute -- is answered from that
    /// record. Nothing is ever re-derived. `FilteredMotionValidator` accepts an edge by
    /// looking it up rather than by re-rolling it, which is where the cost went.
    ///
    /// This is exact, not an approximation: the rollout integrates a control held
    /// constant over each step, so the straight line between two consecutive waypoints
    /// *is* the executed motion. A densified record is therefore a genuinely geometric
    /// path, and sampling strictly inside a step is meaningful rather than invented.
    ///
    /// ### Straight where it can be, filtered where it must be
    ///
    /// A rollout does not step at a fixed rate. Each filter call also hands back how
    /// long the control it returned stays certified -- the span over which nothing the
    /// filter enforces can bind -- and the rollout runs that control for exactly that
    /// long before asking again. Where there is room, one call certifies the whole
    /// extension and the edge is a single straight line, produced without checking
    /// anything along it, because there is nothing along it left to find out. Where
    /// there is not, the certificate is short, the step falls back to `stepSize`, and
    /// the QP does the work as before. Nothing in between is guessed at: the two
    /// regimes are separated by a Lipschitz bound over the interval, not by a
    /// heuristic, so the coarse steps are the *more* trustworthy of the two -- they
    /// hold at every point of the span, where a QP step holds at its endpoints and
    /// leans on the margin in between.
    ///
    /// This is why the certificate is worth having at all: an extension through open
    /// space used to cost one QP solve per 0.025 rad and now costs one evaluation
    /// total. `setMaxStepScale(1)` puts the fixed step back for comparison.
    ///
    /// ### The contract, and where it bends
    ///
    /// OMPL asks that `interpolate(a, b, 1, out)` give `out == b`. That holds here only
    /// when the rollout is unobstructed -- and when it is, this space *is* a
    /// `RealVectorStateSpace`, up to quantisation: the nominal control re-aims at `b`
    /// every step, so in free space the rollout advances exactly `|b - a| / N` per step
    /// and lands on `b`. Near an obstacle the rollout slides along the boundary and
    /// ends somewhere else, which is the entire point: the deflected endpoint is a
    /// perfectly good new tree node, and it is safe, whereas a straight line into the
    /// obstacle is not.
    ///
    /// A planner that stores `interpolate()`'s output as a node is therefore storing a
    /// state the rollout actually produced, and this space has that rollout on file.
    /// `geometric::RRT` does exactly that on its long-extension branch
    /// (`d > maxDistance_`). On the short branch it stores the raw sample instead,
    /// which the rollout may not reach -- that case is what `FilteredMotionValidator`'s
    /// arrive-or-reject fallback is for.
    ///
    /// ### What it does not give you
    ///
    /// `distance()` is still Euclidean, and is now a *lower bound* on the length of
    /// the motion `interpolate()` would actually take, since sliding around an
    /// obstacle is longer than cutting through it. That only makes it a slightly
    /// looser nearest-neighbour heuristic, which is all a tree planner needs it for.
    ///
    /// Not thread safe: the ledger and the statistics are mutable, and `ControlFilter`
    /// already cost thread safety anyway since `CBFControlFilter` solves into shared
    /// scratch.
    class FilteredStateSpace : public base::RealVectorStateSpace
    {
    public:
        using Configuration = ControlFilter::Configuration;
        using Control = ControlFilter::Control;

        static constexpr int dimension = Configuration::RowsAtCompileTime;

        struct Rollout
        {
            Configuration end;                     ///< where the rollout finished
            std::vector<Configuration> waypoints;  ///< the executed motion; front() is the start
            unsigned int steps{0};                 ///< filter calls made
            unsigned int filtered{0};              ///< of those, how many the CBF altered
            unsigned int blocked{0};               ///< of those, how many had no safe control
            unsigned int coarse{0};                ///< of those, how many ran past `stepSize`
            double travel{0.0};                    ///< joint-space radians covered
            double fraction{0.0};                  ///< share of the full horizon it got through
            bool reachedTarget{false};             ///< did it finish within reachTolerance of `to`?
        };

        /// Aggregate counters, so a planner run can be costed without instrumenting
        /// the planner. Mutable because `interpolate()` is const.
        struct Statistics
        {
            std::size_t rollouts{0};
            std::size_t steps{0};  ///< filter calls actually made
            std::size_t filtered{0};
            std::size_t blocked{0};
            std::size_t coarse{0};  ///< steps that ran past `stepSize` on a certificate
            double travel{0.0};     ///< joint-space radians rolled; `travel / steps` is what a
                                    ///< filter call buys, which is the number to quote a cost at
            std::size_t abandoned{0};  ///< rollouts discarded for making no progress
            std::size_t served{0};     ///< queries answered from the ledger, at no filter cost
            std::size_t recorded{0};   ///< edges committed to the ledger
            std::size_t evicted{0};    ///< edges dropped for capacity; should stay zero
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

        /// How close a rollout must finish to `to` for it to count as having got there.
        ///
        /// Only the arrive-or-reject fallback in `FilteredMotionValidator` uses this --
        /// a recorded edge is answered by lookup and needs no tolerance. Exact equality
        /// is the right answer in free space but useless near an obstacle, where two
        /// rollouts from the same state toward slightly different targets deflect
        /// slightly differently. Defaults to one full-speed step.
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

        /// Roll out from \p from toward \p to for \p fraction of the full horizon,
        /// keeping every state it passes through.
        ///
        /// The nominal control re-aims at \p to at every step rather than being held
        /// constant. Both choices reduce to a straight line in free space, but
        /// re-aiming also recovers the original intent after the filter deflects it,
        /// which is what makes a long extension useful rather than merely safe.
        ///
        /// A pure function of (`from`, `to`, `fraction`): no memo, no resumption. What
        /// used to be a prefix cache is now the ledger, which keeps whole edges rather
        /// than one trajectory prefix, and is consulted by `interpolate()` before this
        /// is ever called.
        Rollout roll(const Configuration &from, const Configuration &to, double fraction) const
        {
            const unsigned int total = horizonSteps(from, to);
            const double horizon = static_cast<double>(total) * stepSize_;
            const double budget = std::clamp(fraction, 0.0, 1.0) * horizon;

            // Durations below this are indistinguishable from having arrived, and joint
            // displacements below the second are indistinguishable from zero: at ~1e-12
            // rad even the longest lever arm on the arm moves by a picometre.
            const double negligibleTime = 1e-9 * stepSize_;
            constexpr double negligibleAngle = 1e-12;

            Rollout out;
            out.end = from;
            // Deliberately not `total + 1`: a certified edge holds two waypoints and the
            // vector is moved into the ledger keeping whatever it reserved.
            out.waypoints.reserve(std::min<std::size_t>(total + 1, 16));
            out.waypoints.push_back(from);

            bool terminal = false;
            Control nominal;
            Control applied;
            double elapsed = 0.0;
            while (budget - elapsed > negligibleTime)
            {
                // Time left in the *full* horizon, so a truncated rollout follows the
                // same trajectory as the prefix of a complete one.
                const double remaining = horizon - elapsed;
                for (int j = 0; j < dimension; ++j)
                    nominal[j] = std::clamp((to[j] - out.end[j]) / remaining, -maxSpeed_[j], maxSpeed_[j]);

                double certified = 0.0;
                const ControlFilter::Status status =
                    filter_.filter(out.end, nominal, stepSize_, applied, certified);
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

                // How far to run what the filter just handed back. The floor is the step
                // it was asked about, which it answered for; above that the filter has
                // certified itself a no-op, so running on is not an extrapolation but a
                // saving of calls whose outcome is already known.
                const double span =
                    std::min(std::max(stepSize_, std::min(certified, maxStepScale_ * stepSize_)),
                             budget - elapsed);

                Configuration landing = out.end + applied * span;
                // A hop that runs the horizon out was aimed to finish on `to`, and with
                // nothing in the way it does -- to the last bit. Recognising that lets an
                // unobstructed edge end on the state that was asked for rather than one
                // rounding away from it, which is what the ledger keys on.
                if ((landing - to).cwiseAbs().maxCoeff() <= negligibleAngle)
                    landing = to;

                elapsed += span;
                ++out.steps;
                // A cornered-but-feasible QP answers with a zero control, which is
                // certified for as long as you like and goes nowhere. Charge the call and
                // run the clock out, but keep it out of the record: a repeated waypoint
                // is not a motion.
                if (bitwiseEqual(landing, out.end))
                    continue;

                out.travel += (landing - out.end).norm();
                out.end = landing;
                out.waypoints.push_back(out.end);
                if (span > stepSize_)
                    ++out.coarse;
            }

            if (terminal)
                out.blocked = 1;
            out.fraction = horizon > 0.0 ? elapsed / horizon : 0.0;
            out.reachedTarget =
                budget - elapsed <= negligibleTime && (out.end - to).norm() <= reachTolerance();

            statistics_.rollouts += 1;
            statistics_.filtered += out.filtered;
            statistics_.blocked += out.blocked;
            statistics_.coarse += out.coarse;
            statistics_.travel += out.travel;
            return out;
        }

        Rollout roll(const base::State *from, const base::State *to, double fraction) const
        {
            return roll(configurationOf(from), configurationOf(to), fraction);
        }

        /// A recorded edge, oriented the way it was asked for.
        ///
        /// Valid until the next `record()`, which may rehash or evict. Both call sites
        /// consume it immediately.
        class EdgeRecord
        {
        public:
            EdgeRecord() = default;
            EdgeRecord(const std::vector<Configuration> *waypoints, bool reversed)
              : waypoints_(waypoints), reversed_(reversed)
            {
            }

            explicit operator bool() const
            {
                return waypoints_ != nullptr;
            }

            /// True when this edge was found stored the other way round. Safe to use:
            /// a recorded polyline visits the same states and the same segments in
            /// either direction. It is *not* a rollout run backwards -- the filter is
            /// not invertible -- and it does not need to be, because what is being
            /// asserted is the geometry of the motion, not the control that made it.
            bool reversed() const
            {
                return reversed_;
            }

            std::size_t size() const
            {
                return waypoints_ == nullptr ? 0 : waypoints_->size();
            }

            /// The i-th waypoint in *query* order.
            const Configuration &operator[](std::size_t i) const
            {
                return reversed_ ? (*waypoints_)[waypoints_->size() - 1 - i] : (*waypoints_)[i];
            }

            /// The state at fraction \p t of the edge, linear between waypoints.
            ///
            /// Exact, including strictly inside a step: the control is held constant
            /// over a step, so the segment joining two consecutive waypoints is the
            /// executed motion rather than a stand-in for it. `t = 0` and `t = 1`
            /// return the endpoints bit-exactly.
            Configuration at(double t) const
            {
                const std::size_t count = size();
                if (count == 1)
                    return (*this)[0];

                const double u = std::clamp(t, 0.0, 1.0) * static_cast<double>(count - 1);
                const auto index = static_cast<std::size_t>(u);
                if (index + 1 >= count)
                    return (*this)[count - 1];
                const double fraction = u - static_cast<double>(index);
                return (*this)[index] + fraction * ((*this)[index + 1] - (*this)[index]);
            }

        private:
            const std::vector<Configuration> *waypoints_{nullptr};
            bool reversed_{false};
        };

        /// The trajectory recorded for the edge \p from -> \p to, in either direction,
        /// or a false record if there is none.
        EdgeRecord recordedEdge(const Configuration &from, const Configuration &to) const
        {
            auto found = ledger_.find(Edge{from, to});
            if (found != ledger_.end())
            {
                ++statistics_.served;
                return EdgeRecord(&found->second, false);
            }
            found = ledger_.find(Edge{to, from});
            if (found != ledger_.end())
            {
                ++statistics_.served;
                return EdgeRecord(&found->second, true);
            }
            return EdgeRecord();
        }

        EdgeRecord recordedEdge(const base::State *from, const base::State *to) const
        {
            return recordedEdge(configurationOf(from), configurationOf(to));
        }

        /// Keep \p waypoints as the executed motion of the edge \p from -> \p to.
        ///
        /// Refuses degenerate edges and never overwrites: re-recording a key with a
        /// different polyline would make replay ambiguous, and the first record is the
        /// one the planner built its tree on.
        void record(const Configuration &from, const Configuration &to,
                    std::vector<Configuration> waypoints) const
        {
            if (waypoints.size() < 2 || bitwiseEqual(from, to))
                return;

            const Edge key{from, to};
            if (ledger_.find(key) != ledger_.end())
                return;

            ledgerWaypoints_ += waypoints.size();
            order_.push_back(key);
            ledger_.emplace(key, std::move(waypoints));
            ++statistics_.recorded;
            evictToCapacity();
        }

        /// Does the pending rollout describe the edge \p from -> \p to, either way round?
        ///
        /// `interpolate()` leaves its rollout here rather than in the ledger, because at
        /// that point the planner has not yet decided whether to keep the edge. The
        /// motion validator is the one that knows, and it commits.
        bool staged(const Configuration &from, const Configuration &to) const
        {
            return staged_.valid && ((bitwiseEqual(staged_.from, from) && bitwiseEqual(staged_.to, to)) ||
                                     (bitwiseEqual(staged_.from, to) && bitwiseEqual(staged_.to, from)));
        }

        bool staged(const base::State *from, const base::State *to) const
        {
            return staged(configurationOf(from), configurationOf(to));
        }

        /// Move the pending rollout into the ledger.
        ///
        /// Counts as served: the caller asked about an edge and got an answer that cost
        /// no filter calls. This is the common case during planning -- the validator is
        /// asking about the extension `interpolate()` just produced -- so leaving it out
        /// would make `statistics().served` read zero on a run that never re-rolled
        /// anything.
        void commitStaged() const
        {
            if (staged_.valid)
            {
                record(staged_.from, staged_.to, std::move(staged_.waypoints));
                ++statistics_.served;
            }
            staged_.waypoints.clear();
            staged_.valid = false;
        }

        /// Ledger size limit, in waypoints. Reaching it evicts oldest-first, which
        /// degrades an affected edge back to being re-derived rather than replayed --
        /// so `statistics().evicted` staying zero is part of the contract, not a
        /// nicety. One waypoint is 48 bytes; the default is roughly 50 MB.
        std::size_t ledgerCapacity() const
        {
            return ledgerCapacity_;
        }

        void setLedgerCapacity(std::size_t maxWaypoints)
        {
            ledgerCapacity_ = maxWaypoints;
            evictToCapacity();
        }

        std::size_t ledgerWaypoints() const
        {
            return ledgerWaypoints_;
        }

        std::size_t ledgerEdges() const
        {
            return ledger_.size();
        }

        void clearLedger() const
        {
            ledger_.clear();
            order_.clear();
            ledgerWaypoints_ = 0;
            staged_.waypoints.clear();
            staged_.valid = false;
        }

        /// Longest step the rollout may take, as a multiple of `stepSize`. Unbounded by
        /// default; `1.0` restores the fixed step exactly, which is the A/B.
        ///
        /// A step only exceeds `stepSize` when the filter has certified that it is a
        /// no-op over the whole of it (`ControlFilter::filter()`'s five-argument form),
        /// so the cap buys nothing in safety -- it exists to isolate the effect when
        /// measuring, and to keep waypoint spacing bounded for a consumer that wants it.
        double maxStepScale() const
        {
            return maxStepScale_;
        }

        void setMaxStepScale(double scale)
        {
            if (scale < 1.0)
                throw Exception("FilteredStateSpace: maxStepScale must be at least 1");
            maxStepScale_ = scale;
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
            // from may alias state, so finish reading before writing.
            const Configuration a = configurationOf(from);
            const Configuration b = configurationOf(to);

            // A recorded edge is a reconstruction, never an extension: replay it and
            // skip both the progress test (which only guards the creation of new edges)
            // and enforceBounds (every waypoint was in bounds when it was recorded).
            if (const EdgeRecord record = recordedEdge(a, b))
            {
                setState(state, record.at(t));
                return;
            }

            Rollout rollout = roll(a, b, t);
            staged_.waypoints.clear();
            staged_.valid = false;

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
            // rather than a timeout. It matters more now, not less: the goal tree
            // accepts far more extensions than it used to.
            //
            // Note the threshold is relative to the progress a *free-space* rollout
            // would have made for this `t`, not absolute.
            const double span = (b - a).norm();
            const double target = span * std::clamp(t, 0.0, 1.0);
            const double achieved = span - (b - rollout.end).norm();
            if (achieved < minProgressFraction_ * target)
            {
                ++statistics_.abandoned;
                setState(state, a);
                enforceBounds(state);
                return;
            }

            setState(state, rollout.end);
            enforceBounds(state);

            // The recorded endpoint has to be bit-identical to the state the planner
            // will hand back, or the lookup misses forever. Clamping essentially never
            // happens -- the filter already respects the joint limits the bounds are
            // set from -- and when it does, the clamp displacement is not something the
            // filter certified, so the honest move is to not record and let the edge be
            // re-derived the old way.
            const Configuration stored = configurationOf(state);
            if (bitwiseEqual(stored, rollout.end))
            {
                staged_.from = a;
                staged_.to = stored;
                staged_.waypoints = std::move(rollout.waypoints);
                staged_.valid = true;
            }
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

        /// Bitwise equality, because the point is to recognise a state the planner just
        /// handed back unmodified -- `copyState` is a memcpy, so the bits survive. Not
        /// `operator==`, which calls -0.0 and 0.0 equal while their bit patterns differ;
        /// hashing and equality have to agree. A near-miss is a miss, which only costs
        /// a rollout.
        static bool bitwiseEqual(const Configuration &a, const Configuration &b)
        {
            return std::memcmp(a.data(), b.data(), sizeof(double) * dimension) == 0;
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
        struct Edge
        {
            Configuration from;
            Configuration to;
        };

        struct EdgeHash
        {
            std::size_t operator()(const Edge &edge) const
            {
                return hashConfiguration(edge.to, hashConfiguration(edge.from, 0xcbf29ce484222325ull));
            }
        };

        struct EdgeEqual
        {
            bool operator()(const Edge &a, const Edge &b) const
            {
                return bitwiseEqual(a.from, b.from) && bitwiseEqual(a.to, b.to);
            }
        };

        /// The rollout `interpolate()` last produced, held until the motion validator
        /// says whether the planner is keeping the edge. Extensions that are rejected
        /// therefore cost nothing in memory.
        struct Staged
        {
            Configuration from{Configuration::Zero()};
            Configuration to{Configuration::Zero()};
            std::vector<Configuration> waypoints;
            bool valid{false};
        };

        static std::size_t hashConfiguration(const Configuration &q, std::uint64_t seed)
        {
            std::uint64_t hash = seed;
            for (int j = 0; j < dimension; ++j)
            {
                std::uint64_t word = 0;
                std::memcpy(&word, q.data() + j, sizeof(word));
                hash = (hash ^ word) * 0x100000001b3ull;
            }
            return static_cast<std::size_t>(hash);
        }

        void evictToCapacity() const
        {
            while (ledgerWaypoints_ > ledgerCapacity_ && !order_.empty())
            {
                const auto oldest = ledger_.find(order_.front());
                if (oldest != ledger_.end())
                {
                    ledgerWaypoints_ -= oldest->second.size();
                    ledger_.erase(oldest);
                    ++statistics_.evicted;
                }
                order_.pop_front();
            }
        }

        const ControlFilter &filter_;
        double stepSize_;
        Control maxSpeed_;
        double reachTolerance_{-1.0};
        double maxStepScale_{std::numeric_limits<double>::infinity()};
        double minProgressFraction_{0.25};
        std::size_t ledgerCapacity_{1u << 20};
        mutable Statistics statistics_;
        mutable Staged staged_;
        mutable std::unordered_map<Edge, std::vector<Configuration>, EdgeHash, EdgeEqual> ledger_;
        mutable std::deque<Edge> order_;  ///< insertion order, for eviction
        mutable std::size_t ledgerWaypoints_{0};
    };
}  // namespace ompl::cbf
