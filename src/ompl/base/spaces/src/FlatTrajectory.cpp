/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Clayton W. Ramsey */

#include "ompl/base/spaces/FlatTrajectory.h"

#include "ompl/util/Exception.h"

#include <algorithm>
#include <utility>

namespace ompl::base
{
    namespace
    {
        /** \brief The motions steering through \e states in order, under \e space. */
        std::vector<FlatMotion> steerThrough(const FlatStateSpace *space, const std::vector<const State *> &states)
        {
            if (space == nullptr)
                throw Exception("FlatTrajectory needs a FlatStateSpace");

            std::vector<FlatMotion> motions;
            motions.reserve(states.empty() ? 0u : states.size() - 1u);
            for (std::size_t i = 1; i < states.size(); ++i)
            {
                std::optional<FlatMotion> motion = space->steer(states[i - 1u], states[i]);
                if (motion.has_value())
                    motions.push_back(std::move(*motion));
            }

            return motions;
        }

        /** \brief The flat state space \e path runs through. */
        const FlatStateSpace *flatSpaceOf(const geometric::PathGeometric &path)
        {
            const auto *space = dynamic_cast<const FlatStateSpace *>(path.getSpaceInformation()->getStateSpace().get());
            if (space == nullptr)
                throw Exception("FlatTrajectory needs a path through a FlatStateSpace");
            return space;
        }

        /** \brief The states \e path runs through. */
        std::vector<const State *> statesOf(const geometric::PathGeometric &path)
        {
            std::vector<const State *> states(path.getStateCount());
            for (std::size_t i = 0; i < states.size(); ++i)
                states[i] = path.getState(i);
            return states;
        }
    }  // namespace

    FlatTrajectory::FlatTrajectory(std::vector<FlatMotion> motions) : motions_(std::move(motions))
    {
        starts_.reserve(motions_.size() + 1u);
        double elapsed = 0.;
        for (const FlatMotion &motion : motions_)
        {
            if (motion.outputDimension() != motions_.front().outputDimension())
                throw Exception("FlatTrajectory needs every motion to move through the same dimensions");

            elapsed += motion.duration();
            starts_.push_back(elapsed);
        }
    }

    FlatTrajectory::FlatTrajectory(const FlatStateSpace *space, const std::vector<const State *> &states)
      : FlatTrajectory(steerThrough(space, states))
    {
    }

    FlatTrajectory::FlatTrajectory(const geometric::PathGeometric &path)
      : FlatTrajectory(flatSpaceOf(path), statesOf(path))
    {
    }

    const FlatMotion &FlatTrajectory::motion(std::size_t index) const
    {
        if (index >= motions_.size())
            throw Exception("FlatTrajectory was asked for a motion it doesn't hold");
        return motions_[index];
    }

    double FlatTrajectory::startTime(std::size_t index) const
    {
        if (index >= starts_.size())
            throw Exception("FlatTrajectory was asked for a start time it doesn't hold");
        return starts_[index];
    }

    double FlatTrajectory::duration() const
    {
        return starts_.back();
    }

    unsigned int FlatTrajectory::outputDimension() const
    {
        return motions_.empty() ? 0u : motions_.front().outputDimension();
    }

    void FlatTrajectory::evaluate(double t, unsigned int level, Eigen::Ref<Eigen::VectorXd> out) const
    {
        if (motions_.empty())
        {
            out.setZero();
            return;
        }

        // The last start at or below t belongs to the motion covering t, and the run ends on the last
        // motion rather than on the entry past it.
        const auto after = std::upper_bound(starts_.begin(), starts_.end(), t);
        const auto index =
            std::min(motions_.size() - 1u, static_cast<std::size_t>(std::distance(starts_.begin(), after)) -
                                               (after == starts_.begin() ? 0u : 1u));

        const FlatMotion &motion = motions_[index];
        motion.evaluate(std::clamp(t - starts_[index], 0., motion.duration()), level, out);
    }

    Eigen::VectorXd FlatTrajectory::evaluate(double t) const
    {
        Eigen::VectorXd value(outputDimension());
        evaluate(t, 0u, value);
        return value;
    }

    void FlatTrajectory::toState(const FlatStateSpace *space, double t, State *state) const
    {
        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < space->getOrder(); ++level)
        {
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            Eigen::Map<Eigen::VectorXd> output(values, space->getOutputDimension());
            evaluate(t, level, output);
        }
    }

}  // namespace ompl::base
