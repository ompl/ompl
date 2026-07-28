#pragma once

#include <algorithm>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/RandomNumbers.h>

namespace ompl::cbf
{
    /// A directed control sampler that actually aims at the target: the control is
    /// the joint velocity that would carry the arm from `source` to `dest` over the
    /// sampled number of steps, clamped to the control bounds.
    ///
    /// OMPL's default `control::SimpleDirectedControlSampler` ignores direction — it
    /// draws k random controls and keeps whichever happens to land nearest. For a
    /// 6-DoF arm that is a very weak steer, and it would also make a CBF comparison
    /// meaningless: random controls rarely aim at an obstacle, so the filter would
    /// seldom engage. Note when reading benchmarks that this sampler is an
    /// improvement over the stock baseline *independently* of any filtering, so it
    /// belongs on both sides of a with/without-CBF comparison.
    ///
    /// Safety is not this class's job. It proposes the nominal intent; the
    /// `FilteredStatePropagator` underneath decides what is actually applied.
    class JointSteeringControlSampler : public control::DirectedControlSampler
    {
    public:
        explicit JointSteeringControlSampler(const control::SpaceInformation *si)
          : control::DirectedControlSampler(si)
          , dimension_(si->getStateSpace()->as<base::RealVectorStateSpace>()->getDimension())
          , bounds_(si->getControlSpace()->as<control::RealVectorControlSpace>()->getBounds())
        {
        }

        unsigned int sampleTo(control::Control *control, const base::State *source,
                              base::State *dest) override
        {
            return steer(control, source, dest);
        }

        /// The previous control carries no useful information for a kinematic arm,
        /// so this is the same steer.
        unsigned int sampleTo(control::Control *control, const control::Control * /*previous*/,
                              const base::State *source, base::State *dest) override
        {
            return steer(control, source, dest);
        }

    private:
        unsigned int steer(control::Control *control, const base::State *source, base::State *dest)
        {
            const auto *si = static_cast<const control::SpaceInformation *>(si_);
            const unsigned int steps =
                rng_.uniformInt(si->getMinControlDuration(), si->getMaxControlDuration());
            const double horizon = steps * si->getPropagationStepSize();

            const double *from = source->as<base::RealVectorStateSpace::StateType>()->values;
            const double *to = dest->as<base::RealVectorStateSpace::StateType>()->values;
            double *values = control->as<control::RealVectorControlSpace::ControlType>()->values;

            // The constant velocity that would close the gap in `horizon` seconds,
            // clamped to what the control space allows.
            for (unsigned int j = 0; j < dimension_; ++j)
                values[j] = std::clamp((to[j] - from[j]) / horizon, bounds_.low[j], bounds_.high[j]);

            // dest must come back as the state actually reached.
            return si->propagateWhileValid(source, control, static_cast<int>(steps), dest);
        }

        unsigned int dimension_;
        base::RealVectorBounds bounds_;
        RNG rng_;
    };
}  // namespace ompl::cbf
