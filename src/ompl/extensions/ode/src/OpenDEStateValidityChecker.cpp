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

#include "ompl/extensions/ode/OpenDEStateValidityChecker.h"
#include "ompl/util/Exception.h"

ompl::control::OpenDEStateValidityChecker::OpenDEStateValidityChecker(const SpaceInformationPtr &si)
  : base::StateValidityChecker(si)
{
    if (dynamic_cast<OpenDEStateSpace *>(si->getStateSpace().get()) == nullptr)
        throw Exception("Cannot create state validity checking for OpenDE without OpenDE state space");
    osm_ = si->getStateSpace()->as<OpenDEStateSpace>();
}

bool ompl::control::OpenDEStateValidityChecker::isValid(const base::State *state) const
{
    const auto *s = state->as<OpenDEStateSpace::StateType>();

    // if we know the value of the validity flag for this state, we return it
    if ((s->collision & (1 << OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT)) != 0)
        return (s->collision & (1 << OpenDEStateSpace::STATE_VALIDITY_VALUE_BIT)) != 0;

    // if not, we compute it:
    bool valid = false;

    if (!osm_->evaluateCollision(state))
        valid = osm_->satisfiesBoundsExceptRotation(s);

    if (valid)
        s->collision &= (1 << OpenDEStateSpace::STATE_VALIDITY_VALUE_BIT);

    // mark the fact we know the value of the validity bit
    s->collision &= (1 << OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT);

    return valid;
}
