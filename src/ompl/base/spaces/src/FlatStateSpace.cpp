/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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

/* Author: Clayton Ramsey */

#include "ompl/base/spaces/FlatStateSpace.h"

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/util/Exception.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ompl::base
{
    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, unsigned int order)
    {
        if (order != 2u)
            throw Exception("FlatStateSpace only supports order 2");

        checkComponent(output, 0u, "flat output");
        outputDimension_ = output->getDimension();

        setName("Flat" + output->getName());
        type_ = STATE_SPACE_FLAT;

        addSubspace(output, 1.);
        for (unsigned int level = 1; level < order; ++level)
            addSubspace(std::make_shared<RealVectorStateSpace>(outputDimension_), 1.);
        lock();

        setDefaultWeights();
        declareParams();
    }

    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, const std::vector<StateSpacePtr> &derivatives)
    {
        if (derivatives.size() != 1u)
            throw Exception("FlatStateSpace only supports order 2, so it needs exactly one derivative space");

        checkComponent(output, 0u, "flat output");
        outputDimension_ = output->getDimension();

        setName("Flat" + output->getName());
        type_ = STATE_SPACE_FLAT;

        addSubspace(output, 1.);
        for (const StateSpacePtr &derivative : derivatives)
        {
            checkComponent(derivative, outputDimension_, "derivative level");
            addSubspace(derivative, 1.);
        }
        lock();

        setDefaultWeights();
        declareParams();
    }

    void FlatStateSpace::declareParams()
    {
        params_.declareParam<double>("rho", [this](double value) { setRho(value); }, [this] { return getRho(); });
    }

    void FlatStateSpace::checkComponent(const StateSpacePtr &space, unsigned int dimension, const char *role)
    {
        if (space == nullptr)
            throw Exception(std::string("FlatStateSpace needs a ") + role + " space");
        if (dynamic_cast<const RealVectorStateSpace *>(space.get()) == nullptr)
            throw Exception(std::string("FlatStateSpace needs a ") + role +
                            " space deriving from RealVectorStateSpace");
        if (space->getDimension() < 1u)
            throw Exception(std::string("FlatStateSpace needs a ") + role + " space of at least one dimension");
        if (dimension > 0u && space->getDimension() != dimension)
            throw Exception(std::string("FlatStateSpace needs its ") + role +
                            " space to match the flat output dimension");
    }

    void FlatStateSpace::setDefaultWeights()
    {
        for (unsigned int i = 0; i < componentCount_; ++i)
        {
            const double extent = components_[i]->getMaximumExtent();
            setSubspaceWeight(i, extent > std::numeric_limits<double>::epsilon() ? 1. / extent : 1.);
        }
        defaultWeights_ = weights_;
    }

    void FlatStateSpace::setDerivativeBound(unsigned int level, double bound)
    {
        if (level < 1u || level >= componentCount_)
            throw Exception("FlatStateSpace derivative levels run from 1 through the order minus one");
        if (!(bound > 0.))
            throw Exception("FlatStateSpace needs a positive derivative bound");

        getDerivativeSpace(level)->setBounds(-bound, bound);
    }

    void FlatStateSpace::toFlatState(const State *state, Eigen::Ref<Eigen::MatrixXd> flatState) const
    {
        const auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < componentCount_; ++level)
        {
            const double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int axis = 0; axis < outputDimension_; ++axis)
                flatState(level, axis) = values[axis];
        }
    }

    void FlatStateSpace::fromFlatState(const Eigen::Ref<const Eigen::MatrixXd> &flatState, State *state) const
    {
        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < componentCount_; ++level)
        {
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int axis = 0; axis < outputDimension_; ++axis)
                values[axis] = flatState(level, axis);
        }
    }

    std::optional<FlatMotion> FlatStateSpace::steer(const State *from, const State *to) const
    {
        Eigen::MatrixXd start(componentCount_, outputDimension_);
        Eigen::MatrixXd finish(componentCount_, outputDimension_);
        toFlatState(from, start);
        toFlatState(to, finish);
        return steering_.steer(start, finish);
    }

    void FlatStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
    {
        bool firstTime = true;
        std::optional<FlatMotion> motion;
        interpolate(from, to, t, firstTime, motion, state);
    }

    void FlatStateSpace::interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     std::optional<FlatMotion> &motion, State *state) const
    {
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }

        // Steering reads both flat states before anything is written, so state can alias one of them.
        if (firstTime)
        {
            motion = steer(from, to);
            firstTime = false;
        }

        if (!motion.has_value())
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        interpolate(*motion, t, state);
    }

    void FlatStateSpace::interpolate(const FlatMotion &motion, double t, State *state) const
    {
        // Each component holds its values contiguously, so the motion writes its derivative levels
        // straight into the state and nothing is allocated per sample.
        const double time = t * motion.duration();
        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < componentCount_; ++level)
        {
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            Eigen::Map<Eigen::VectorXd> output(values, outputDimension_);
            motion.evaluate(time, level, output);
        }
    }

    unsigned int FlatStateSpace::validSegmentCount(const State *state1, const State *state2) const
    {
        const std::optional<FlatMotion> motion = steer(state1, state2);
        return motion.has_value() ? validSegmentCount(*motion) : 1u;
    }

    unsigned int FlatStateSpace::validSegmentCount(const FlatMotion &motion) const
    {
        const double span = motion.duration() * motion.peakSpeed();
        const auto count = static_cast<unsigned int>(std::ceil(span / longestValidSegment_));
        return std::max(1u, longestValidSegmentCountFactor_ * count);
    }

    void FlatStateSpace::registerProjections()
    {
        registerDefaultProjection(std::make_shared<SubspaceProjectionEvaluator>(this, 0u));
    }

    void FlatStateSpace::setup()
    {
        // Bounds usually arrive after construction, so the weights get another look here.
        // A caller who set weights of their own keeps them.
        if (weights_ == defaultWeights_)
            setDefaultWeights();

        CompoundStateSpace::setup();
    }

    State *FlatStateSpace::allocState() const
    {
        auto *state = new StateType();
        allocStateComponents(state);
        return state;
    }
}  // namespace ompl::base
