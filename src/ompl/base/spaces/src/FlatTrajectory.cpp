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
        /** \brief The flat state space \e path runs through. */
        FlatStateSpacePtr flatSpaceOf(const geometric::PathGeometric &path)
        {
            auto space = std::dynamic_pointer_cast<FlatStateSpace>(path.getSpaceInformation()->getStateSpace());
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

    FlatTrajectory::FlatTrajectory(FlatStateSpacePtr space, const std::vector<const State *> &states)
      : space_(std::move(space))
    {
        if (space_ == nullptr)
            throw Exception("FlatTrajectory needs a FlatStateSpace");
        if (states.empty())
            return;

        const unsigned int dimension = space_->getOutputDimension();
        Eigen::MatrixXd first(space_->getOrder(), dimension);
        space_->toFlatState(states.front(), first);

        motions_.reserve(states.size() - 1u);
        waypoints_.reserve(states.size());
        offsets_.reserve(states.size());
        starts_.reserve(states.size());

        Eigen::VectorXd offset = first.row(0).transpose();
        double elapsed = 0.;
        for (std::size_t i = 1; i < states.size(); ++i)
        {
            std::optional<FlatMotion> motion = space_->steer(states[i - 1u], states[i]);
            if (!motion.has_value())
                continue;

            waypoints_.emplace_back(space_);
            waypoints_.back() = states[i - 1u];
            offsets_.push_back(offset);

            // Adding up the coordinates of each motion in turn keeps level 0 continuous across the joins,
            // where the flat output itself might wrap.
            offset += motion->evaluate(motion->duration());
            elapsed += motion->duration();
            starts_.push_back(elapsed);
            motions_.push_back(std::move(*motion));
        }

        waypoints_.emplace_back(space_);
        waypoints_.back() = states.back();
        offsets_.push_back(offset);
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

    std::size_t FlatTrajectory::motionAt(double t) const
    {
        // The last start at or below t belongs to the motion covering t, and the run ends on the last
        // motion rather than on the entry past it.
        const auto after = std::upper_bound(starts_.begin(), starts_.end(), t);
        return std::min(motions_.size() - 1u, static_cast<std::size_t>(std::distance(starts_.begin(), after)) -
                                                  (after == starts_.begin() ? 0u : 1u));
    }

    void FlatTrajectory::evaluate(double t, unsigned int level, Eigen::Ref<Eigen::VectorXd> out) const
    {
        if (motions_.empty())
        {
            out.setZero();
            return;
        }

        const std::size_t index = motionAt(t);
        const FlatMotion &motion = motions_[index];
        motion.evaluate(std::clamp(t - starts_[index], 0., motion.duration()), level, out);
        if (level == 0u)
            out += offsets_[index];
    }

    Eigen::VectorXd FlatTrajectory::evaluate(double t) const
    {
        Eigen::VectorXd value(outputDimension());
        evaluate(t, 0u, value);
        return value;
    }

    void FlatTrajectory::toState(double t, State *state) const
    {
        if (waypoints_.empty())
            throw Exception("FlatTrajectory holds no states to write");

        if (motions_.empty())
        {
            space_->copyState(state, waypoints_.front().get());
            return;
        }

        const std::size_t index = motionAt(t);
        const FlatMotion &motion = motions_[index];
        const double fraction = std::clamp((t - starts_[index]) / motion.duration(), 0., 1.);
        space_->interpolate(waypoints_[index].get(), motion, fraction, state);
    }
}  // namespace ompl::base
