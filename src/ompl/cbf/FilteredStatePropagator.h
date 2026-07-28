#pragma once

#include <cstddef>

#include <Eigen/Core>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/ControlFilter.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl::cbf
{
    /// Kinematic joint-velocity propagation with a safety filter in the loop.
    ///
    /// The nominal dynamics are `q(t + dt) = q(t) + u dt`. Before integrating,
    /// the control is passed through a `ControlFilter`, so the propagated system is
    /// the *closed-loop* one: the robot together with the safety filter it would
    /// actually run. That is the point of doing it here rather than in a steer
    /// function — planning happens for the system that will be deployed.
    ///
    /// ### Why this is the right seam for CBF steering
    ///
    /// An OMPL control edge is one constant control applied for n steps:
    /// `control::SpaceInformation::propagateWhileValid` loops over
    /// `propagate(state, control, dt, …)` with the same control each time. CBF
    /// steering needs a different control at every step, because the barrier rows
    /// are rebuilt at each state — so a filtered multi-step rollout cannot be
    /// returned from a `DirectedControlSampler`. Filtering *inside* propagation
    /// keeps the stored control equal to the nominal intent, with the filter
    /// folded into the dynamics. The reached state stays a deterministic function
    /// of (state, nominal control, steps), which is exactly OMPL's edge model, so
    /// every stock control planner works unmodified and
    /// `control::PathControl::check()` — which re-propagates and compares — passes.
    ///
    /// ### Assumptions
    ///
    /// The state space must be a `base::RealVectorStateSpace` and the control space
    /// a `control::RealVectorControlSpace`, both of dimension
    /// `ControlFilter::Configuration::RowsAtCompileTime` (6, for the UR5). This is
    /// checked at construction.
    ///
    /// ### The state validity checker becomes redundant
    ///
    /// This is the point of the whole exercise, so it is worth stating plainly:
    /// when the filter certifies every step it produces, there is nothing left for
    /// a `base::StateValidityChecker` to do, and it should be removed —
    ///
    ///     si->setStateValidityChecker(
    ///         std::make_shared<base::AllValidStateValidityChecker>(si));
    ///
    /// Keeping a collision checker alongside a CBF filter means paying for the
    /// same geometry twice per step: the barrier already evaluates every sphere
    /// against the field to build its constraint rows, and `isSafe()` then repeats
    /// the forward kinematics and the distance queries to learn something the
    /// barrier already knew.
    ///
    /// What you give up by removing it, all of which the caller now owns:
    ///
    /// - **Anything outside the barrier's model.** The checker was the last line
    ///   of defence against errors in the sphere approximation, the SDF, or the
    ///   step linearization. With it gone, `ClearanceBarrier`'s margin is the only
    ///   thing covering them. Audit a returned path (interpolate it and evaluate
    ///   the barrier at every step) rather than assuming.
    /// - **Start-state validation.** `Planner::checkValidity()` will now accept a
    ///   start state in collision. Check it yourself before planning.
    /// - **Truncation on `Blocked`.** See below — though a collision checker never
    ///   caught that case either.
    ///
    /// ### Blocked steps
    ///
    /// When the filter reports `Blocked` there is no safe control, and this
    /// propagator holds the state still rather than inventing motion. Note the
    /// consequence: because the held state is the (valid) state it started from,
    /// `propagateWhileValid` will not truncate the edge — it will run to its full
    /// step count having gone nowhere. That is safe but wasteful, and it is the
    /// caller's job to notice; `statistics().blocked` counts it, and
    /// `propagateReporting()` returns the status directly.
    class FilteredStatePropagator : public control::StatePropagator
    {
    public:
        using Configuration = ControlFilter::Configuration;
        using Control = ControlFilter::Control;

        static constexpr int dimension = Configuration::RowsAtCompileTime;

        /// How often the filter engaged. Useful for benchmarking: a run where
        /// `filtered` is near zero did not exercise the CBF at all.
        struct Statistics
        {
            std::size_t propagations{0};
            std::size_t unchanged{0};
            std::size_t filtered{0};
            std::size_t blocked{0};
        };

        /// \p filter is not copied and must outlive this propagator.
        FilteredStatePropagator(control::SpaceInformation *si, const ControlFilter &filter)
          : control::StatePropagator(si), filter_(filter)
        {
            checkSpaces(si);
        }

        FilteredStatePropagator(const control::SpaceInformationPtr &si, const ControlFilter &filter)
          : control::StatePropagator(si), filter_(filter)
        {
            checkSpaces(si.get());
        }

        void propagate(const base::State *state, const control::Control *control, double duration,
                       base::State *result) const override
        {
            propagateReporting(state, control, duration, result);
        }

        /// Propagation that also says what the filter did. Same semantics as
        /// `propagate()` otherwise.
        ControlFilter::Status propagateReporting(const base::State *state, const control::Control *control,
                                                 double duration, base::State *result) const
        {
            // The caller is allowed to pass result == state, so read everything out
            // before writing anything back.
            const auto *from = state->as<base::RealVectorStateSpace::StateType>();
            const auto *nominalControl = control->as<control::RealVectorControlSpace::ControlType>();

            Configuration q;
            Control nominal;
            for (int j = 0; j < dimension; ++j)
            {
                q[j] = from->values[j];
                nominal[j] = nominalControl->values[j];
            }

            ++statistics_.propagations;

            Control applied;
            const ControlFilter::Status status = filter_.filter(q, nominal, duration, applied);
            switch (status)
            {
                case ControlFilter::Status::Unchanged:
                    ++statistics_.unchanged;
                    break;
                case ControlFilter::Status::Filtered:
                    ++statistics_.filtered;
                    break;
                case ControlFilter::Status::Blocked:
                    ++statistics_.blocked;
                    // No safe control: stay put rather than invent motion.
                    applied.setZero();
                    break;
            }

            auto *to = result->as<base::RealVectorStateSpace::StateType>();
            for (int j = 0; j < dimension; ++j)
                to->values[j] = q[j] + applied[j] * duration;

            // The filter's joint-limit box already keeps this inside bounds when it
            // is enabled; this makes the invariant hold either way.
            si_->getStateSpace()->enforceBounds(result);
            return status;
        }

        /// Projection is not invertible, so a filtered step cannot be undone.
        bool canPropagateBackward() const override
        {
            return false;
        }

        const ControlFilter &filter() const
        {
            return filter_;
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
        void checkSpaces(const control::SpaceInformation *si) const
        {
            if (si->getStateSpace()->as<base::RealVectorStateSpace>()->getDimension() !=
                static_cast<unsigned int>(dimension))
                throw Exception("FilteredStatePropagator: state space dimension must be 6");
            if (si->getControlSpace()->as<control::RealVectorControlSpace>()->getDimension() !=
                static_cast<unsigned int>(dimension))
                throw Exception("FilteredStatePropagator: control space dimension must be 6");
        }

        const ControlFilter &filter_;
        mutable Statistics statistics_;
    };
}  // namespace ompl::cbf
