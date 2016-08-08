/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/geometric/HillClimbing.h"

namespace ompl
{
    namespace magic
    {
        /** \brief Maximum number of consecutive failures to allow
            before giving up on improving a state. A failure consists
            of being unable to sample a state that is closer to the
            specified goal region.*/
        static const unsigned int MAX_CLIMB_NO_UPDATE_STEPS = 10;
    }
}

bool ompl::geometric::HillClimbing::tryToImprove(const base::GoalRegion &goal, base::State *state, double nearDistance,
                                                 double *betterGoalDistance) const
{
    double tempDistance;
    double initialDistance;

    bool wasValid = valid(state);
    bool wasValidStart = wasValid;

    bool wasSatisfied = goal.isSatisfied(state, &initialDistance);
    bool wasSatisfiedStart = wasSatisfied;

    double bestDist = initialDistance;

    base::StateSamplerPtr ss = si_->allocStateSampler();
    base::State *test = si_->allocState();
    unsigned int noUpdateSteps = 0;

    for (unsigned int i = 0; noUpdateSteps < magic::MAX_CLIMB_NO_UPDATE_STEPS && i < maxImproveSteps_; ++i)
    {
        bool update = false;
        ss->sampleUniformNear(test, state, nearDistance);
        bool isValid = valid(test);
        bool isSatisfied = goal.isSatisfied(test, &tempDistance);
        if (!wasValid && isValid)
        {
            si_->copyState(state, test);
            wasValid = true;
            wasSatisfied = isSatisfied;
            update = true;
        }
        else if (wasValid == isValid)
        {
            if (!wasSatisfied && isSatisfied)
            {
                si_->copyState(state, test);
                wasSatisfied = true;
                update = true;
            }
            else if (wasSatisfied == isSatisfied)
            {
                if (tempDistance < bestDist)
                {
                    si_->copyState(state, test);
                    bestDist = tempDistance;
                    update = true;
                }
            }
        }
        if (update)
            noUpdateSteps = 0;
        else
            noUpdateSteps++;
    }
    si_->freeState(test);

    if (betterGoalDistance)
        *betterGoalDistance = bestDist;
    return (bestDist < initialDistance) || (!wasSatisfiedStart && wasSatisfied) || (!wasValidStart && wasValid);
}
