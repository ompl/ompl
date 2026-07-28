#pragma once

#include <utility>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/cbf/FilteredStateSpace.h>

namespace ompl::cbf
{
    /// The motion validator that goes with `FilteredStateSpace`: a motion from s1 to
    /// s2 is valid when the CBF rollout from s1 actually arrives at s2.
    ///
    /// Note what is *not* being checked. Every state a rollout produces is certified by
    /// the barrier as it is produced, so there is no collision checking here and no
    /// `StateValidityChecker` is consulted — the question "is this motion safe?" has
    /// already been answered yes by construction. The only open question is whether the
    /// motion goes where the planner asked, and that is what this reports.
    ///
    /// That distinction is what makes the pair work with a stock geometric planner.
    /// `geometric::RRT` has two extension branches:
    ///
    /// - **Long** (`d > maxDistance_`): it calls `interpolate()` and stores the result.
    ///   The node is the rollout's own endpoint, so this validator re-rolls and arrives
    ///   — the extension is accepted wherever the rollout got to, including when the
    ///   filter deflected it around an obstacle. No edge is ever wasted, which is the
    ///   property control-space planning could not offer.
    /// - **Short** (`d <= maxDistance_`): it stores the raw sample. The rollout reaches
    ///   that only if the way there is clear, so this validator correctly rejects it
    ///   otherwise. Without the reach test the planner would happily add nodes it
    ///   cannot actually drive to.
    ///
    /// `reachTolerance` is genuinely load-bearing on the long branch: `interpolate(a, r,
    /// frac)` aims at the random sample `r`, while `checkMotion(a, x)` aims at the
    /// endpoint `x` it produced. Same start, same obstacle, marginally different target,
    /// so the two rollouts deflect marginally differently and exact equality would
    /// reject perfectly good extensions. See `FilteredStateSpace::reachTolerance()`.
    ///
    /// That second rollout is also the long branch's whole cost, and when the filter did
    /// not engage it is redundant — `FilteredStateSpace::certifiedByLastRollout()` says
    /// when, and this consults it first. So the tolerance above matters exactly in the
    /// case that still rolls: the one where the filter deflected something.
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
            // The rollout the planner just took to *produce* s2 already answers this;
            // see FilteredStateSpace::certifiedByLastRollout().
            const bool reached =
                space_->certifiedByLastRollout(s1, s2) || space_->roll(s1, s2, 1.0).reachedTarget;
            reached ? ++valid_ : ++invalid_;
            return reached;
        }

        bool checkMotion(const base::State *s1, const base::State *s2,
                         std::pair<base::State *, double> &lastValid) const override
        {
            if (space_->certifiedByLastRollout(s1, s2))
            {
                // lastValid is only ever read on failure, so there is nothing to fill in.
                ++valid_;
                return true;
            }

            const FilteredStateSpace::Rollout rollout = space_->roll(s1, s2, 1.0);
            if (rollout.reachedTarget)
            {
                ++valid_;
                return true;
            }

            ++invalid_;
            // Where the rollout got to *is* reachable and safe, so it is a strictly
            // better "last valid state" than a point on the straight line would be.
            if (lastValid.first != nullptr)
                FilteredStateSpace::setState(lastValid.first, rollout.end);
            const unsigned int total =
                space_->horizonSteps(FilteredStateSpace::configurationOf(s1),
                                     FilteredStateSpace::configurationOf(s2));
            lastValid.second = total == 0 ? 0.0 : static_cast<double>(rollout.steps) / total;
            return false;
        }

    private:
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
