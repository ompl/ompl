#pragma once

#include <utility>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/cbf/FilteredStateSpace.h>

namespace ompl::cbf
{
    /// The motion validator that goes with `FilteredStateSpace`: a motion is valid when
    /// the space has the rollout that produced it on file.
    ///
    /// Note what is *not* being checked. Every state a rollout produces is certified by
    /// the barrier as it is produced, so there is no collision checking here and no
    /// `StateValidityChecker` is consulted -- the question "is this motion safe?" has
    /// already been answered yes by construction. The only open question is whether the
    /// planner's edge corresponds to a motion this space actually produced, and that is
    /// a lookup.
    ///
    /// That is the whole cost saving. Previously this re-ran the rollout aimed at the
    /// stored endpoint, to find out whether the edge could be reproduced from its two
    /// endpoints alone -- roughly doubling the filter calls on any extension the CBF
    /// deflected. Nothing needs reproducing now: the trajectory is kept, so the edge is
    /// answered rather than re-derived. See `FilteredStateSpace`'s header.
    ///
    /// The three ways a motion gets an answer, in the order they are tried:
    ///
    /// - **Staged.** `interpolate()` just produced this edge and left its rollout
    ///   pending. This is the common case for `geometric::RRT`'s long-extension branch
    ///   (`d > maxDistance_`) and for both of `geometric::RRTConnect`'s trees. Commit
    ///   the record and accept. No filter calls.
    /// - **Recorded.** The edge is already in the ledger, in either direction. This
    ///   covers re-queries such as `geometric::PathGeometric::check()`, and it covers
    ///   `RRTConnect`'s goal tree, which validates its edges *backwards*
    ///   (`RRTConnect.cpp:143`, `checkMotion(dstate, nmotion->state)`). A recorded
    ///   polyline visits the same states and the same segments either way round, so a
    ///   reversed hit is a genuine answer -- and it is one the old re-rolling validator
    ///   could never give, which is why deflected goal-tree extensions used to be
    ///   rejected outright.
    /// - **Neither.** Roll for real and require arrival. This is `geometric::RRT`'s
    ///   short branch (`d <= maxDistance_`), where the target is the raw sample rather
    ///   than something a rollout produced. The rollout reaches it only if the way there
    ///   is clear, so arrive-or-reject is the correct answer; without it the planner
    ///   would add nodes it cannot drive to. `reachTolerance` is what "arrived" means,
    ///   and this fallback is now the only place it is used.
    class FilteredMotionValidator : public base::MotionValidator
    {
    public:
        explicit FilteredMotionValidator(base::SpaceInformation *si)
          : base::MotionValidator(si), space_(spaceOf(si))
        {
        }

        explicit FilteredMotionValidator(const base::SpaceInformationPtr &si)
          : base::MotionValidator(si), space_(spaceOf(si.get()))
        {
        }

        bool checkMotion(const base::State *s1, const base::State *s2) const override
        {
            if (zeroLength(s1, s2))
            {
                ++invalid_;
                return false;
            }
            if (accepted(s1, s2))
            {
                ++valid_;
                return true;
            }

            FilteredStateSpace::Rollout rollout = space_->roll(s1, s2, 1.0);
            if (!rollout.reachedTarget)
            {
                ++invalid_;
                return false;
            }

            recordArrival(s1, s2, std::move(rollout));
            ++valid_;
            return true;
        }

        bool checkMotion(const base::State *s1, const base::State *s2,
                         std::pair<base::State *, double> &lastValid) const override
        {
            if (zeroLength(s1, s2))
            {
                ++invalid_;
                if (lastValid.first != nullptr)
                    si_->copyState(lastValid.first, s1);
                lastValid.second = 0.0;
                return false;
            }
            if (accepted(s1, s2))
            {
                // lastValid is only ever read on failure, so there is nothing to fill in.
                ++valid_;
                return true;
            }

            FilteredStateSpace::Rollout rollout = space_->roll(s1, s2, 1.0);
            if (rollout.reachedTarget)
            {
                recordArrival(s1, s2, std::move(rollout));
                ++valid_;
                return true;
            }

            ++invalid_;
            // Where the rollout got to *is* reachable and safe, so it is a strictly
            // better "last valid state" than a point on the straight line would be.
            if (lastValid.first != nullptr)
                FilteredStateSpace::setState(lastValid.first, rollout.end);
            // How far through the motion it got, which the rollout reports directly: with
            // a certified step the count of filter calls no longer measures progress.
            lastValid.second = rollout.fraction;
            return false;
        }

    private:
        /// `geometric::RRT` has no equal-states guard of its own, unlike
        /// `RRTConnect.cpp:132`. So when `interpolate()` reports an extension as going
        /// nowhere, RRT asks whether the state can reach *itself* -- a zero-horizon
        /// rollout, which trivially "arrives" and would earn a duplicate node with a
        /// zero-length edge. Refuse it here instead.
        static bool zeroLength(const base::State *s1, const base::State *s2)
        {
            return FilteredStateSpace::bitwiseEqual(FilteredStateSpace::configurationOf(s1),
                                                    FilteredStateSpace::configurationOf(s2));
        }

        /// Valid by construction: either the pending rollout produced this edge, or the
        /// ledger already holds it. Commits the former, so only edges the planner keeps
        /// are stored.
        bool accepted(const base::State *s1, const base::State *s2) const
        {
            if (space_->staged(s1, s2))
            {
                space_->commitStaged();
                return true;
            }
            return static_cast<bool>(space_->recordedEdge(s1, s2));
        }

        /// Record a fallback rollout that arrived.
        ///
        /// The rollout finishes within `reachTolerance` of \p s2 rather than exactly on
        /// it, while the planner stores \p s2, so the last waypoint is snapped to what
        /// the planner will hold. That leaves one final segment of at most one
        /// full-speed step which the filter did not certify -- which is not new, it is
        /// exactly the gap `reachedTarget` has always accepted. Recording it makes the
        /// gap explicit and bounded instead of implicit.
        void recordArrival(const base::State *s1, const base::State *s2,
                           FilteredStateSpace::Rollout rollout) const
        {
            const FilteredStateSpace::Configuration from = FilteredStateSpace::configurationOf(s1);
            const FilteredStateSpace::Configuration to = FilteredStateSpace::configurationOf(s2);
            if (rollout.waypoints.size() < 2)
                return;
            rollout.waypoints.back() = to;
            space_->record(from, to, std::move(rollout.waypoints));
        }

        static const FilteredStateSpace *spaceOf(const base::SpaceInformation *si)
        {
            const auto *space = dynamic_cast<const FilteredStateSpace *>(si->getStateSpace().get());
            if (space == nullptr)
                throw Exception("FilteredMotionValidator requires a FilteredStateSpace");
            return space;
        }

        const FilteredStateSpace *space_;
    };
}  // namespace ompl::cbf
