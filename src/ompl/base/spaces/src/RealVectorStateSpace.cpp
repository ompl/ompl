/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorStateProjections.h"
#include "ompl/util/Exception.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>

void ompl::base::RealVectorStateSampler::sampleUniform(State *state)
{
    const unsigned int dim = space_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateSpace *>(space_)->getBounds();

    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    for (unsigned int i = 0; i < dim; ++i)
        rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::base::RealVectorStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    const unsigned int dim = space_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateSpace *>(space_)->getBounds();

    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    const auto *rnear = static_cast<const RealVectorStateSpace::StateType *>(near);
    for (unsigned int i = 0; i < dim; ++i)
        rstate->values[i] = rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - distance),
                                             std::min(bounds.high[i], rnear->values[i] + distance));
}

void ompl::base::RealVectorStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    const unsigned int dim = space_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateSpace *>(space_)->getBounds();

    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    const auto *rmean = static_cast<const RealVectorStateSpace::StateType *>(mean);
    for (unsigned int i = 0; i < dim; ++i)
    {
        double v = rng_.gaussian(rmean->values[i], stdDev);
        if (v < bounds.low[i])
            v = bounds.low[i];
        else if (v > bounds.high[i])
            v = bounds.high[i];
        rstate->values[i] = v;
    }
}

void ompl::base::RealVectorStateSpace::registerProjections()
{
    // compute a default random projection
    if (dimension_ > 0)
    {
        if (dimension_ > 2)
        {
            int p = std::max(2, (int)ceil(log((double)dimension_)));
            registerDefaultProjection(std::make_shared<RealVectorRandomLinearProjectionEvaluator>(this, p));
        }
        else
            registerDefaultProjection(std::make_shared<RealVectorIdentityProjectionEvaluator>(this));
    }
}

void ompl::base::RealVectorStateSpace::setup()
{
    bounds_.check();
    StateSpace::setup();
}

void ompl::base::RealVectorStateSpace::addDimension(const std::string &name, double minBound, double maxBound)
{
    addDimension(minBound, maxBound);
    setDimensionName(dimension_ - 1, name);
}

void ompl::base::RealVectorStateSpace::addDimension(double minBound, double maxBound)
{
    dimension_++;
    stateBytes_ = dimension_ * sizeof(double);
    bounds_.low.push_back(minBound);
    bounds_.high.push_back(maxBound);
    dimensionNames_.resize(dimension_, "");
}

void ompl::base::RealVectorStateSpace::setBounds(const RealVectorBounds &bounds)
{
    bounds.check();
    if (bounds.low.size() != dimension_)
        throw Exception("Bounds do not match dimension of state space: expected dimension " +
                        std::to_string(dimension_) + " but got dimension " + std::to_string(bounds.low.size()));
    bounds_ = bounds;
}

void ompl::base::RealVectorStateSpace::setBounds(double low, double high)
{
    RealVectorBounds bounds(dimension_);
    bounds.setLow(low);
    bounds.setHigh(high);
    setBounds(bounds);
}

unsigned int ompl::base::RealVectorStateSpace::getDimension() const
{
    return dimension_;
}

const std::string &ompl::base::RealVectorStateSpace::getDimensionName(unsigned int index) const
{
    if (index < dimensionNames_.size())
        return dimensionNames_[index];
    throw Exception("Index out of bounds");
}

int ompl::base::RealVectorStateSpace::getDimensionIndex(const std::string &name) const
{
    auto it = dimensionIndex_.find(name);
    return it != dimensionIndex_.end() ? (int)it->second : -1;
}

void ompl::base::RealVectorStateSpace::setDimensionName(unsigned int index, const std::string &name)
{
    if (index < dimensionNames_.size())
    {
        dimensionNames_[index] = name;
        dimensionIndex_[name] = index;
    }
    else
        throw Exception("Cannot set dimension name. Index out of bounds");
}

double ompl::base::RealVectorStateSpace::getMaximumExtent() const
{
    double e = 0.0;
    for (unsigned int i = 0; i < dimension_; ++i)
    {
        double d = bounds_.high[i] - bounds_.low[i];
        e += d * d;
    }
    return sqrt(e);
}

double ompl::base::RealVectorStateSpace::getMeasure() const
{
    double m = 1.0;
    for (unsigned int i = 0; i < dimension_; ++i)
    {
        m *= bounds_.high[i] - bounds_.low[i];
    }
    return m;
}

void ompl::base::RealVectorStateSpace::enforceBounds(State *state) const
{
    auto *rstate = static_cast<StateType *>(state);
    for (unsigned int i = 0; i < dimension_; ++i)
    {
        if (rstate->values[i] > bounds_.high[i])
            rstate->values[i] = bounds_.high[i];
        else if (rstate->values[i] < bounds_.low[i])
            rstate->values[i] = bounds_.low[i];
    }
}

bool ompl::base::RealVectorStateSpace::satisfiesBounds(const State *state) const
{
    const auto *rstate = static_cast<const StateType *>(state);
    for (unsigned int i = 0; i < dimension_; ++i)
        if (rstate->values[i] - std::numeric_limits<double>::epsilon() > bounds_.high[i] ||
            rstate->values[i] + std::numeric_limits<double>::epsilon() < bounds_.low[i])
            return false;
    return true;
}

void ompl::base::RealVectorStateSpace::copyState(State *destination, const State *source) const
{
    memcpy(static_cast<StateType *>(destination)->values, static_cast<const StateType *>(source)->values, stateBytes_);
}

unsigned int ompl::base::RealVectorStateSpace::getSerializationLength() const
{
    return stateBytes_;
}

void ompl::base::RealVectorStateSpace::serialize(void *serialization, const State *state) const
{
    memcpy(serialization, state->as<StateType>()->values, stateBytes_);
}

void ompl::base::RealVectorStateSpace::deserialize(State *state, const void *serialization) const
{
    memcpy(state->as<StateType>()->values, serialization, stateBytes_);
}

double ompl::base::RealVectorStateSpace::distance(const State *state1, const State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const StateType *>(state1)->values;
    const double *s2 = static_cast<const StateType *>(state2)->values;

    for (unsigned int i = 0; i < dimension_; ++i)
    {
        double diff = (*s1++) - (*s2++);
        dist += diff * diff;
    }
    return sqrt(dist);
}

bool ompl::base::RealVectorStateSpace::equalStates(const State *state1, const State *state2) const
{
    const double *s1 = static_cast<const StateType *>(state1)->values;
    const double *s2 = static_cast<const StateType *>(state2)->values;
    for (unsigned int i = 0; i < dimension_; ++i)
    {
        double diff = (*s1++) - (*s2++);
        if (fabs(diff) > std::numeric_limits<double>::epsilon() * 2.0)
            return false;
    }
    return true;
}

void ompl::base::RealVectorStateSpace::interpolate(const State *from, const State *to, const double t,
                                                   State *state) const
{
    const auto *rfrom = static_cast<const StateType *>(from);
    const auto *rto = static_cast<const StateType *>(to);
    const StateType *rstate = static_cast<StateType *>(state);
    for (unsigned int i = 0; i < dimension_; ++i)
        rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
}

ompl::base::StateSamplerPtr ompl::base::RealVectorStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<RealVectorStateSampler>(this);
}

ompl::base::State *ompl::base::RealVectorStateSpace::allocState() const
{
    auto *rstate = new StateType();
    rstate->values = new double[dimension_];
    return rstate;
}

void ompl::base::RealVectorStateSpace::freeState(State *state) const
{
    auto *rstate = static_cast<StateType *>(state);
    delete[] rstate->values;
    delete rstate;
}

double *ompl::base::RealVectorStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return index < dimension_ ? static_cast<StateType *>(state)->values + index : nullptr;
}

void ompl::base::RealVectorStateSpace::printState(const State *state, std::ostream &out) const
{
    out << "RealVectorState [";
    if (state != nullptr)
    {
        const auto *rstate = static_cast<const StateType *>(state);
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            out << rstate->values[i];
            if (i + 1 < dimension_)
                out << ' ';
        }
    }
    else
        out << "nullptr" << std::endl;
    out << ']' << std::endl;
}

void ompl::base::RealVectorStateSpace::printSettings(std::ostream &out) const
{
    out << "Real vector state space '" << getName() << "' of dimension " << dimension_ << " with bounds: " << std::endl;
    out << "  - min: ";
    for (unsigned int i = 0; i < dimension_; ++i)
        out << bounds_.low[i] << " ";
    out << std::endl;
    out << "  - max: ";
    for (unsigned int i = 0; i < dimension_; ++i)
        out << bounds_.high[i] << " ";
    out << std::endl;

    bool printNames = false;
    for (unsigned int i = 0; i < dimension_; ++i)
        if (!dimensionNames_[i].empty())
            printNames = true;
    if (printNames)
    {
        out << "  and dimension names: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            out << "'" << dimensionNames_[i] << "' ";
        out << std::endl;
    }
}
