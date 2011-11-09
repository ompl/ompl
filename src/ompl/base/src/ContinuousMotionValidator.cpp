/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Ryan Luna */

#include "ompl/base/ContinuousMotionValidator.h"
#include "ompl/util/Exception.h"

void ompl::base::ContinuousMotionValidator::defaultSettings(void)
{
    stateSpace_ = si_->getStateSpace().get();
    if (!stateSpace_)
        throw Exception("No state space for motion validator");
}

bool ompl::base::ContinuousMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
{
    bool valid = false;
    // if there is a collision, collisionTime will contain the time to collision,
    // parameterized from [0,1], where s1 is 0 and s2 is 1.
    double collisionTime;
    valid = si_->getStateValidityChecker ()->isValid (s1, s2, collisionTime);

    // Find the last valid state before collision...
    // NOTE: This should probably be refactored so that the continuous checker
    // returns the last valid time.  Last valid transformation may also be
    // possible, but that introduces a dependency on the geometry.
    if (!valid)
    {
        State *lastValidState;
        if (lastValid.first) 
            lastValidState = lastValid.first;
        else 
            lastValidState = si_->allocState ();

        stateSpace_->interpolate (s1, s2, collisionTime, lastValidState);

        while (!si_->isValid (lastValidState) && collisionTime > 0)
        {
            collisionTime -= 0.01;
            stateSpace_->interpolate (s1, s2, collisionTime, lastValidState);
        }

        // ensure that collisionTime is greater than zero
        if (collisionTime < 0.01)
        {
            collisionTime = 0.0;
            si_->copyState (lastValidState, s1);
        }

        lastValid.second = collisionTime;

        if (!lastValid.first)
            si_->freeState (lastValidState);
    }

    // Increment valid/invalid motion counters
    valid ? valid_++ : invalid_++;

    return valid;
}

bool ompl::base::ContinuousMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    double unused;

    // assume motion starts in a valid configuration so s1 is valid
    // Must check validity of s2 before performing collision check between s1 and s2
    bool valid = si_->isValid(s2) && si_->getStateValidityChecker ()->isValid (s1, s2, unused);

    // Increment valid/invalid motion counters
    valid ? valid_++ : invalid_++;

    return valid;
}
