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

#include "ompl/extensions/ode/OpenDEStatePropagator.h"
#include "ompl/extensions/ode/OpenDEStateSpace.h"
#include "ompl/extensions/ode/OpenDEControlSpace.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"

ompl::control::OpenDEStatePropagator::OpenDEStatePropagator(const SpaceInformationPtr &si) : StatePropagator(si)
{
    if (auto *oss = dynamic_cast<OpenDEStateSpace *>(si->getStateSpace().get()))
        env_ = oss->getEnvironment();
    else
        throw Exception("OpenDE State Space needed for OpenDEStatePropagator");
}

/// @cond IGNORE
namespace ompl
{
    struct CallbackParam
    {
        const control::OpenDEEnvironment *env;
        bool collision;
    };

    void nearCallback(void *data, dGeomID o1, dGeomID o2)
    {
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

        if ((b1 != nullptr) && (b2 != nullptr) && (dAreConnectedExcluding(b1, b2, dJointTypeContact) != 0))
            return;

        auto *cp = reinterpret_cast<CallbackParam *>(data);

        const unsigned int maxContacts = cp->env->getMaxContacts(o1, o2);
        if (maxContacts <= 0)
            return;

        auto *contact = new dContact[maxContacts];

        for (unsigned int i = 0; i < maxContacts; ++i)
            cp->env->setupContact(o1, o2, contact[i]);

        if (int numc = dCollide(o1, o2, maxContacts, &contact[0].geom, sizeof(dContact)))
        {
            for (int i = 0; i < numc; ++i)
            {
                dJointID c = dJointCreateContact(cp->env->world_, cp->env->contactGroup_, contact + i);
                dJointAttach(c, b1, b2);
                bool valid = cp->env->isValidCollision(o1, o2, contact[i]);
                if (!valid)
                    cp->collision = true;
                if (cp->env->verboseContacts_)
                {
                    OMPL_DEBUG("%s contact between %s and %s", (valid ? "Valid" : "Invalid"),
                               cp->env->getGeomName(o1).c_str(), cp->env->getGeomName(o1).c_str());
                }
            }
        }

        delete[] contact;
    }
}
/// @endcond

void ompl::control::OpenDEStatePropagator::propagate(const base::State *state, const Control *control,
                                                     const double duration, base::State *result) const
{
    env_->mutex_.lock();

    // place the OpenDE world at the start state
    si_->getStateSpace()->as<OpenDEStateSpace>()->writeState(state);

    // apply the controls
    env_->applyControl(control->as<RealVectorControlSpace::ControlType>()->values);

    // created contacts as needed
    CallbackParam cp = {env_.get(), false};
    for (auto &collisionSpace : env_->collisionSpaces_)
        dSpaceCollide(collisionSpace, &cp, &nearCallback);

    // propagate one step forward
    dWorldQuickStep(env_->world_, (dReal)duration);

    // remove created contacts
    dJointGroupEmpty(env_->contactGroup_);

    // read the final state from the OpenDE world
    si_->getStateSpace()->as<OpenDEStateSpace>()->readState(result);

    env_->mutex_.unlock();

    // update the collision flag for the start state, if needed
    if ((state->as<OpenDEStateSpace::StateType>()->collision & (1 << OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT)) == 0)
    {
        if (cp.collision)
            state->as<OpenDEStateSpace::StateType>()->collision &= (1 << OpenDEStateSpace::STATE_COLLISION_VALUE_BIT);
        state->as<OpenDEStateSpace::StateType>()->collision &= (1 << OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT);
    }
}

bool ompl::control::OpenDEStatePropagator::canPropagateBackward() const
{
    return false;
}
