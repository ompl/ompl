#pragma once

#include <limits>

#include <Eigen/Core>

#include <ompl/robots/UR5.h>

namespace ompl::cbf
{
    /// A safety filter on controls: given where the robot is and what control it
    /// *wants* to apply, return a control it may actually apply.
    ///
    /// This plays the role `base::MotionValidator` plays for geometric planning,
    /// except that it *repairs* rather than merely rejects. A rejecting checker
    /// makes a colliding local step disappear; a filter bends it into a step that
    /// slides along the obstacle instead, so the expansion still makes progress.
    ///
    /// The intended use is inside a `control::StatePropagator`, which is what
    /// makes the arrangement fit OMPL's edge model: an OMPL control edge is a
    /// *single constant control applied for n steps*, so a rollout that varies its
    /// control per step cannot be handed back from a `DirectedControlSampler`.
    /// Filtering inside propagation instead means the edge stores the *nominal*
    /// control and the filter is part of the dynamics — and the reached state
    /// stays a deterministic function of (state, nominal control, steps), exactly
    /// what OMPL assumes. `control::PathControl::check()` re-propagates and
    /// compares, so a deterministic filter makes path verification free.
    ///
    /// Implementations are therefore required to be **deterministic**: the same
    /// (q, nominal) must always yield the same output.
    ///
    /// Working in plain Eigen vectors rather than `base::State` / `Control` keeps
    /// filters testable on their own; the propagator does the conversion.
    class ControlFilter
    {
    public:
        using Configuration = robots::UR5::Configuration;
        /// Joint velocities. Same shape as a configuration, different meaning.
        using Control = robots::UR5::Configuration;

        enum class Status
        {
            /// The nominal control was already safe and was passed through as-is.
            Unchanged,
            /// The nominal control was modified to keep the step safe.
            Filtered,
            /// No safe control exists from here; the caller must not take the step.
            /// Implementations write a zero control in this case.
            Blocked
        };

        static const char *statusString(Status status)
        {
            switch (status)
            {
                case Status::Unchanged:
                    return "unchanged";
                case Status::Filtered:
                    return "filtered";
                case Status::Blocked:
                    return "blocked";
            }
            return "unknown";
        }

        ControlFilter() = default;
        virtual ~ControlFilter() = default;

        ControlFilter(const ControlFilter &) = delete;
        ControlFilter &operator=(const ControlFilter &) = delete;

        /// Map a nominal control to a safe one at configuration \p q, for a step of
        /// length \p duration.
        ///
        /// \p duration is a property of the call, not of the filter: how far the
        /// control will be integrated determines how much clearance the step can
        /// consume, so a filter configured with its own step length could silently
        /// certify a different step than the one actually taken. Non-positive
        /// durations are not supported (see `canPropagateBackward`).
        virtual Status filter(const Configuration &q, const Control &nominal, double duration,
                              Control &filtered) const = 0;

        /// As above, additionally reporting how long the control it hands back stays
        /// certified: the longest duration for which applying \p filtered from \p q is
        /// provably a *no-op* for this filter, because nothing it enforces can bind
        /// before then.
        ///
        /// A caller may therefore integrate \p filtered over any span up to \p certified
        /// and get exactly the motion repeated filtering would have produced, without
        /// paying for the intervening calls. It is a statement about the whole interval,
        /// not just its end -- the guarantee holds at every point along the way -- so it
        /// is a certificate rather than an extrapolation.
        ///
        /// Zero, the default, means "no certificate available"; the caller then takes
        /// the step it asked about and calls again, which is what it did before this
        /// existed. Implementations must never report a duration they cannot back:
        /// under-reporting costs calls, over-reporting costs safety.
        virtual Status filter(const Configuration &q, const Control &nominal, double duration,
                              Control &filtered, double &certified) const
        {
            certified = 0.0;
            return filter(q, nominal, duration, filtered);
        }

        /// Human-readable name, for logging and benchmark labels.
        virtual const char *name() const = 0;
    };

    /// A filter that does nothing. This is the A/B baseline: dropped into the same
    /// propagator as a real filter, it recovers ordinary unfiltered propagation, so
    /// "with CBF" and "without CBF" runs differ in exactly one object.
    class PassthroughFilter : public ControlFilter
    {
    public:
        Status filter(const Configuration & /*q*/, const Control &nominal, double /*duration*/,
                      Control &filtered) const override
        {
            filtered = nominal;
            return Status::Unchanged;
        }

        /// A filter that enforces nothing is a no-op for any duration whatsoever, so the
        /// honest certificate is unbounded. The A/B baseline therefore takes each edge in
        /// a single straight step, which is precisely what "no filter" should cost.
        Status filter(const Configuration & /*q*/, const Control &nominal, double /*duration*/,
                      Control &filtered, double &certified) const override
        {
            filtered = nominal;
            certified = std::numeric_limits<double>::infinity();
            return Status::Unchanged;
        }

        const char *name() const override
        {
            return "passthrough";
        }
    };
}  // namespace ompl::cbf
