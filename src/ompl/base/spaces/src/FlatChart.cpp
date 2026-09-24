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

#include "ompl/base/spaces/FlatChart.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/util/Exception.h"

#include <boost/math/constants/constants.hpp>

#include <cmath>
#include <utility>

namespace ompl::base
{
    void RealVectorFlatChart::difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const
    {
        const double *start = from->as<RealVectorStateSpace::StateType>()->values;
        const double *finish = to->as<RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < dimension_; ++i)
            out[i] = finish[i] - start[i];
    }

    void RealVectorFlatChart::advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta,
                                      State *out) const
    {
        const double *start = from->as<RealVectorStateSpace::StateType>()->values;
        double *finish = out->as<RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < dimension_; ++i)
            finish[i] = start[i] + delta[i];
    }

    SO2FlatChart::SO2FlatChart() : FlatChart(1u)
    {
        periods_[0] = boost::math::constants::two_pi<double>();
    }

    void SO2FlatChart::difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const
    {
        const double pi = boost::math::constants::pi<double>();

        // Both angles lie in the space, so they're less than a full turn apart and one wrap is enough.
        double turn = to->as<SO2StateSpace::StateType>()->value - from->as<SO2StateSpace::StateType>()->value;
        if (turn >= pi)
            turn -= 2. * pi;
        else if (turn < -pi)
            turn += 2. * pi;
        out[0] = turn;
    }

    void SO2FlatChart::advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta, State *out) const
    {
        const double pi = boost::math::constants::pi<double>();

        // This matches SO2StateSpace::enforceBounds, so an advanced angle satisfies the bounds of the space.
        double angle = std::fmod(from->as<SO2StateSpace::StateType>()->value + delta[0], 2. * pi);
        if (angle < -pi)
            angle += 2. * pi;
        else if (angle >= pi)
            angle -= 2. * pi;
        out->as<SO2StateSpace::StateType>()->value = angle;
    }

    CompoundFlatChart::CompoundFlatChart(std::vector<FlatChartPtr> components)
      : FlatChart(0u), components_(std::move(components))
    {
        offsets_.reserve(components_.size());
        for (const FlatChartPtr &component : components_)
        {
            if (component == nullptr)
                throw Exception("CompoundFlatChart needs a chart for every component");
            offsets_.push_back(static_cast<Eigen::Index>(dimension_));
            dimension_ += component->getDimension();
            periods_.insert(periods_.end(), component->getPeriods().begin(), component->getPeriods().end());
        }
    }

    void CompoundFlatChart::difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const
    {
        const auto *start = from->as<CompoundState>();
        const auto *finish = to->as<CompoundState>();
        for (std::size_t i = 0; i < components_.size(); ++i)
            components_[i]->difference(start->components[i], finish->components[i],
                                       out.segment(offsets_[i], components_[i]->getDimension()));
    }

    void CompoundFlatChart::advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta, State *out) const
    {
        const auto *start = from->as<CompoundState>();
        auto *finish = out->as<CompoundState>();
        for (std::size_t i = 0; i < components_.size(); ++i)
            components_[i]->advance(start->components[i], delta.segment(offsets_[i], components_[i]->getDimension()),
                                    finish->components[i]);
    }

    FlatChartPtr allocFlatChart(const StateSpacePtr &space)
    {
        if (space == nullptr)
            throw Exception("allocFlatChart needs a state space");

        // Casting rather than reading the type catches subclasses, which keep the value layout the chart
        // reads.
        if (dynamic_cast<const RealVectorStateSpace *>(space.get()) != nullptr)
            return std::make_shared<RealVectorFlatChart>(space->getDimension());
        if (dynamic_cast<const SO2StateSpace *>(space.get()) != nullptr)
            return std::make_shared<SO2FlatChart>();

        if (space->isCompound())
        {
            const auto *compound = space->as<CompoundStateSpace>();
            std::vector<FlatChartPtr> components;
            components.reserve(compound->getSubspaceCount());
            for (unsigned int i = 0; i < compound->getSubspaceCount(); ++i)
                components.push_back(allocFlatChart(compound->getSubspace(i)));
            return std::make_shared<CompoundFlatChart>(std::move(components));
        }

        throw Exception("There's no FlatChart for " + space->getName() +
                        ", so FlatStateSpace needs one handed to it at construction");
    }
}  // namespace ompl::base
