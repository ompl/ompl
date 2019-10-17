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

#include "ompl/extensions/ode/OpenDEStateSpace.h"
#include "ompl/util/Console.h"
#include <limits>
#include <queue>
#include <utility>

ompl::control::OpenDEStateSpace::OpenDEStateSpace(OpenDEEnvironmentPtr env, double positionWeight, double linVelWeight,
                                                  double angVelWeight, double orientationWeight)
  : env_(std::move(env))
{
    setName("OpenDE" + getName());
    type_ = base::STATE_SPACE_TYPE_COUNT + 1;
    for (unsigned int i = 0; i < env_->stateBodies_.size(); ++i)
    {
        std::string body = ":B" + std::to_string(i);

        addSubspace(std::make_shared<base::RealVectorStateSpace>(3), positionWeight);  // position
        components_.back()->setName(components_.back()->getName() + body + ":position");

        addSubspace(std::make_shared<base::RealVectorStateSpace>(3), linVelWeight);  // linear velocity
        components_.back()->setName(components_.back()->getName() + body + ":linvel");

        addSubspace(std::make_shared<base::RealVectorStateSpace>(3), angVelWeight);  // angular velocity
        components_.back()->setName(components_.back()->getName() + body + ":angvel");

        addSubspace(std::make_shared<base::SO3StateSpace>(), orientationWeight);  // orientation
        components_.back()->setName(components_.back()->getName() + body + ":orientation");
    }
    lock();
    setDefaultBounds();
}

void ompl::control::OpenDEStateSpace::setDefaultBounds()
{
    // limit all velocities to 1 m/s, 1 rad/s, respectively
    base::RealVectorBounds bounds1(3);
    bounds1.setLow(-1);
    bounds1.setHigh(1);
    setLinearVelocityBounds(bounds1);
    setAngularVelocityBounds(bounds1);

    // find the bounding box that contains all geoms included in the collision spaces
    double mX, mY, mZ, MX, MY, MZ;
    mX = mY = mZ = std::numeric_limits<double>::infinity();
    MX = MY = MZ = -std::numeric_limits<double>::infinity();
    bool found = false;

    std::queue<dSpaceID> spaces;
    for (auto &collisionSpace : env_->collisionSpaces_)
        spaces.push(collisionSpace);

    while (!spaces.empty())
    {
        dSpaceID space = spaces.front();
        spaces.pop();

        int n = dSpaceGetNumGeoms(space);

        for (int j = 0; j < n; ++j)
        {
            dGeomID geom = dSpaceGetGeom(space, j);
            if (dGeomIsSpace(geom) != 0)
                spaces.push((dSpaceID)geom);
            else
            {
                bool valid = true;
                dReal aabb[6];
                dGeomGetAABB(geom, aabb);

                // things like planes are infinite; we want to ignore those
                for (double k : aabb)
                    if (fabs(k) >= std::numeric_limits<dReal>::max())
                    {
                        valid = false;
                        break;
                    }
                if (valid)
                {
                    found = true;
                    if (aabb[0] < mX)
                        mX = aabb[0];
                    if (aabb[1] > MX)
                        MX = aabb[1];
                    if (aabb[2] < mY)
                        mY = aabb[2];
                    if (aabb[3] > MY)
                        MY = aabb[3];
                    if (aabb[4] < mZ)
                        mZ = aabb[4];
                    if (aabb[5] > MZ)
                        MZ = aabb[5];
                }
            }
        }
    }

    if (found)
    {
        double dx = MX - mX;
        double dy = MY - mY;
        double dz = MZ - mZ;
        double dM = std::max(dx, std::max(dy, dz));

        // add 10% in each dimension + 1% of the max dimension
        dx = dx / 10.0 + dM / 100.0;
        dy = dy / 10.0 + dM / 100.0;
        dz = dz / 10.0 + dM / 100.0;

        bounds1.low[0] = mX - dx;
        bounds1.high[0] = MX + dx;
        bounds1.low[1] = mY - dy;
        bounds1.high[1] = MY + dy;
        bounds1.low[2] = mZ - dz;
        bounds1.high[2] = MZ + dz;

        setVolumeBounds(bounds1);
    }
}

void ompl::control::OpenDEStateSpace::copyState(base::State *destination, const base::State *source) const
{
    CompoundStateSpace::copyState(destination, source);
    destination->as<StateType>()->collision = source->as<StateType>()->collision;
}

namespace ompl
{
    /// @cond IGNORE
    struct CallbackParam
    {
        const control::OpenDEEnvironment *env;
        bool collision;
    };

    static void nearCallback(void *data, dGeomID o1, dGeomID o2)
    {
        // if a collision has not already been detected
        if (!reinterpret_cast<CallbackParam *>(data)->collision)
        {
            dBodyID b1 = dGeomGetBody(o1);
            dBodyID b2 = dGeomGetBody(o2);
            if ((b1 != nullptr) && (b2 != nullptr) && (dAreConnectedExcluding(b1, b2, dJointTypeContact) != 0))
                return;

            dContact contact[1];  // one contact is sufficient
            int numc = dCollide(o1, o2, 1, &contact[0].geom, sizeof(dContact));

            // check if there is really a collision
            if (numc != 0)
            {
                // check if the collision is allowed
                bool valid = reinterpret_cast<CallbackParam *>(data)->env->isValidCollision(o1, o2, contact[0]);
                reinterpret_cast<CallbackParam *>(data)->collision = !valid;
                if (reinterpret_cast<CallbackParam *>(data)->env->verboseContacts_)
                {
                    OMPL_DEBUG("%s contact between %s and %s", (valid ? "Valid" : "Invalid"),
                               reinterpret_cast<CallbackParam *>(data)->env->getGeomName(o1).c_str(),
                               reinterpret_cast<CallbackParam *>(data)->env->getGeomName(o2).c_str());
                }
            }
        }
    }
    /// @endcond
}

bool ompl::control::OpenDEStateSpace::evaluateCollision(const base::State *state) const
{
    if ((state->as<StateType>()->collision & (1 << STATE_COLLISION_KNOWN_BIT)) != 0)
        return (state->as<StateType>()->collision & (1 << STATE_COLLISION_VALUE_BIT)) != 0;
    env_->mutex_.lock();
    writeState(state);
    CallbackParam cp = {env_.get(), false};
    for (unsigned int i = 0; !cp.collision && i < env_->collisionSpaces_.size(); ++i)
        dSpaceCollide(env_->collisionSpaces_[i], &cp, &nearCallback);
    env_->mutex_.unlock();
    if (cp.collision)
        state->as<StateType>()->collision &= (1 << STATE_COLLISION_VALUE_BIT);
    state->as<StateType>()->collision &= (1 << STATE_COLLISION_KNOWN_BIT);
    return cp.collision;
}

bool ompl::control::OpenDEStateSpace::satisfiesBoundsExceptRotation(const StateType *state) const
{
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (i % 4 != 3)
            if (!components_[i]->satisfiesBounds(state->components[i]))
                return false;
    return true;
}

void ompl::control::OpenDEStateSpace::setVolumeBounds(const base::RealVectorBounds &bounds)
{
    for (unsigned int i = 0; i < env_->stateBodies_.size(); ++i)
        components_[i * 4]->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::control::OpenDEStateSpace::setLinearVelocityBounds(const base::RealVectorBounds &bounds)
{
    for (unsigned int i = 0; i < env_->stateBodies_.size(); ++i)
        components_[i * 4 + 1]->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::control::OpenDEStateSpace::setAngularVelocityBounds(const base::RealVectorBounds &bounds)
{
    for (unsigned int i = 0; i < env_->stateBodies_.size(); ++i)
        components_[i * 4 + 2]->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

ompl::base::State *ompl::control::OpenDEStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::control::OpenDEStateSpace::freeState(base::State *state) const
{
    CompoundStateSpace::freeState(state);
}

// this function should most likely not be used with OpenDE propagations, but just in case it is called, we need to make
// sure the collision information
// is cleared from the resulting state
void ompl::control::OpenDEStateSpace::interpolate(const base::State *from, const base::State *to, const double t,
                                                  base::State *state) const
{
    CompoundStateSpace::interpolate(from, to, t, state);
    state->as<StateType>()->collision = 0;
}

/// @cond IGNORE
namespace ompl
{
    namespace control
    {
        // we need to make sure any collision information is cleared when states are sampled (just in case this ever
        // happens)
        class WrapperForOpenDESampler : public ompl::base::StateSampler
        {
        public:
            WrapperForOpenDESampler(const base::StateSpace *space, base::StateSamplerPtr wrapped)
              : base::StateSampler(space), wrapped_(std::move(wrapped))
            {
            }

            void sampleUniform(ompl::base::State *state) override
            {
                wrapped_->sampleUniform(state);
                state->as<OpenDEStateSpace::StateType>()->collision = 0;
            }

            void sampleUniformNear(base::State *state, const base::State *near, const double distance) override
            {
                wrapped_->sampleUniformNear(state, near, distance);
                state->as<OpenDEStateSpace::StateType>()->collision = 0;
            }

            void sampleGaussian(base::State *state, const base::State *mean, const double stdDev) override
            {
                wrapped_->sampleGaussian(state, mean, stdDev);
                state->as<OpenDEStateSpace::StateType>()->collision = 0;
            }

        private:
            base::StateSamplerPtr wrapped_;
        };
    }
}
/// @endcond

ompl::base::StateSamplerPtr ompl::control::OpenDEStateSpace::allocDefaultStateSampler() const
{
    base::StateSamplerPtr sampler = base::CompoundStateSpace::allocDefaultStateSampler();
    return std::make_shared<WrapperForOpenDESampler>(this, sampler);
}

ompl::base::StateSamplerPtr ompl::control::OpenDEStateSpace::allocStateSampler() const
{
    base::StateSamplerPtr sampler = base::CompoundStateSpace::allocStateSampler();
    if (dynamic_cast<WrapperForOpenDESampler *>(sampler.get()) != nullptr)
        return sampler;
    return std::make_shared<WrapperForOpenDESampler>(this, sampler);
}

void ompl::control::OpenDEStateSpace::readState(base::State *state) const
{
    auto *s = state->as<StateType>();
    for (int i = (int)env_->stateBodies_.size() - 1; i >= 0; --i)
    {
        unsigned int _i4 = i * 4;

        const dReal *pos = dBodyGetPosition(env_->stateBodies_[i]);
        const dReal *vel = dBodyGetLinearVel(env_->stateBodies_[i]);
        const dReal *ang = dBodyGetAngularVel(env_->stateBodies_[i]);
        double *s_pos = s->as<base::RealVectorStateSpace::StateType>(_i4)->values;
        ++_i4;
        double *s_vel = s->as<base::RealVectorStateSpace::StateType>(_i4)->values;
        ++_i4;
        double *s_ang = s->as<base::RealVectorStateSpace::StateType>(_i4)->values;
        ++_i4;

        for (int j = 0; j < 3; ++j)
        {
            s_pos[j] = pos[j];
            s_vel[j] = vel[j];
            s_ang[j] = ang[j];
        }

        const dReal *rot = dBodyGetQuaternion(env_->stateBodies_[i]);
        base::SO3StateSpace::StateType &s_rot = *s->as<base::SO3StateSpace::StateType>(_i4);

        s_rot.w = rot[0];
        s_rot.x = rot[1];
        s_rot.y = rot[2];
        s_rot.z = rot[3];
    }
    s->collision = 0;
}

void ompl::control::OpenDEStateSpace::writeState(const base::State *state) const
{
    const auto *s = state->as<StateType>();
    for (int i = (int)env_->stateBodies_.size() - 1; i >= 0; --i)
    {
        unsigned int _i4 = i * 4;

        double *s_pos = s->as<base::RealVectorStateSpace::StateType>(_i4)->values;
        ++_i4;
        dBodySetPosition(env_->stateBodies_[i], s_pos[0], s_pos[1], s_pos[2]);

        double *s_vel = s->as<base::RealVectorStateSpace::StateType>(_i4)->values;
        ++_i4;
        dBodySetLinearVel(env_->stateBodies_[i], s_vel[0], s_vel[1], s_vel[2]);

        double *s_ang = s->as<base::RealVectorStateSpace::StateType>(_i4)->values;
        ++_i4;
        dBodySetAngularVel(env_->stateBodies_[i], s_ang[0], s_ang[1], s_ang[2]);

        const base::SO3StateSpace::StateType &s_rot = *s->as<base::SO3StateSpace::StateType>(_i4);
        dQuaternion q;
        q[0] = s_rot.w;
        q[1] = s_rot.x;
        q[2] = s_rot.y;
        q[3] = s_rot.z;
        dBodySetQuaternion(env_->stateBodies_[i], q);
    }
}
