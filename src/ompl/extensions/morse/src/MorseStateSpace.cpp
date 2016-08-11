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

/* Authors: Caleb Voss */

#include "ompl/extensions/morse/MorseStateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"

ompl::base::MorseStateSpace::MorseStateSpace(const MorseEnvironmentPtr &env, double positionWeight, double linVelWeight,
                                             double angVelWeight, double orientationWeight)
  : CompoundStateSpace(), env_(env)
{
    setName("Morse" + getName());
    type_ = STATE_SPACE_TYPE_COUNT + 1;
    for (unsigned int i = 0; i < env_->rigidBodies_; ++i)
    {
        std::string body = ":B" + std::to_string(i);

        addSubspace(std::make_shared<RealVectorStateSpace>(3), positionWeight);  // position
        components_.back()->setName(components_.back()->getName() + body + ":position");

        addSubspace(std::make_shared<RealVectorStateSpace>(3), linVelWeight);  // linear velocity
        components_.back()->setName(components_.back()->getName() + body + ":linvel");

        addSubspace(std::make_shared<RealVectorStateSpace>(3), angVelWeight);  // angular velocity
        components_.back()->setName(components_.back()->getName() + body + ":angvel");

        addSubspace(std::make_shared<SO3StateSpace>(), orientationWeight);  // orientation
        components_.back()->setName(components_.back()->getName() + body + ":orientation");
    }
    // Add the goal region satisfaction flag as a subspace.
    addSubspace(std::make_shared<DiscreteStateSpace>(0, 1), 0.01);
    components_.back()->setName(components_.back()->getName() + ":goalRegionSat");

    lock();
    setBounds();
}

void ompl::base::MorseStateSpace::setBounds()
{
    RealVectorBounds pbounds(3), lbounds(3), abounds(3);
    for (unsigned int i = 0; i < 3; i++)
    {
        pbounds.low[i] = env_->positionBounds_[2 * i];
        pbounds.high[i] = env_->positionBounds_[2 * i + 1];
        lbounds.low[i] = env_->linvelBounds_[2 * i];
        lbounds.high[i] = env_->linvelBounds_[2 * i + 1];
        abounds.low[i] = env_->angvelBounds_[2 * i];
        abounds.high[i] = env_->angvelBounds_[2 * i + 1];
    }
    setPositionBounds(pbounds);
    setLinearVelocityBounds(lbounds);
    setAngularVelocityBounds(abounds);
}

void ompl::base::MorseStateSpace::copyState(State *destination, const State *source) const
{
    CompoundStateSpace::copyState(destination, source);
}

bool ompl::base::MorseStateSpace::satisfiesBounds(const State *state) const
{
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        // for each body, check all bounds except the rotation
        if (i % 4 != 3)
        {
            if (!components_[i]->satisfiesBounds(state->as<CompoundStateSpace::StateType>()->components[i]))
                return false;
        }
    }
    return true;
}

void ompl::base::MorseStateSpace::setPositionBounds(const RealVectorBounds &bounds)
{
    for (unsigned int i = 0; i < env_->rigidBodies_; ++i)
        components_[i * 4]->as<RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::base::MorseStateSpace::setLinearVelocityBounds(const RealVectorBounds &bounds)
{
    for (unsigned int i = 0; i < env_->rigidBodies_; ++i)
        components_[i * 4 + 1]->as<RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::base::MorseStateSpace::setAngularVelocityBounds(const RealVectorBounds &bounds)
{
    for (unsigned int i = 0; i < env_->rigidBodies_; ++i)
        components_[i * 4 + 2]->as<RealVectorStateSpace>()->setBounds(bounds);
}

ompl::base::State *ompl::base::MorseStateSpace::allocState() const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return static_cast<State *>(state);
}

void ompl::base::MorseStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

// this function should most likely not be used with MORSE propagations
void ompl::base::MorseStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    CompoundStateSpace::interpolate(from, to, t, state);
}

ompl::base::StateSamplerPtr ompl::base::MorseStateSpace::allocDefaultStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}

ompl::base::StateSamplerPtr ompl::base::MorseStateSpace::allocStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}

void ompl::base::MorseStateSpace::readState(State *state) const
{
    env_->readState(state);
    for (unsigned int i = 0; i < env_->rigidBodies_ * 4; i += 4)
    {
        // for each body, ensure its rotation is normalized
        SO3StateSpace::StateType *quat = state->as<StateType>()->as<SO3StateSpace::StateType>(i + 3);
        getSubspace(i + 3)->as<SO3StateSpace>()->enforceBounds(quat);
    }
}

void ompl::base::MorseStateSpace::writeState(const State *state) const
{
    env_->writeState(state);
}
