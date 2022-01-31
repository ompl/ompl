/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#include "ompl/base/samplers/ConditionalStateSampler.h"

ompl::base::ConditionalStateSampler::ConditionalStateSampler(const ompl::base::SpaceInformation *si,
                                                             ompl::base::Motion *&startMotion,
                                                             std::vector<Motion *> &goalMotions,
                                                             std::vector<Motion *> &newBatchGoalMotions,
                                                             bool &sampleOldBatch)
  : ValidStateSampler(si)
  , startMotion_(startMotion)
  , goalMotions_(goalMotions)
  , newBatchGoalMotions_(newBatchGoalMotions)
  , sampleOldBatch_(sampleOldBatch)
{
    name_ = "ConditionalSampler";
}

bool ompl::base::ConditionalStateSampler::sample(ompl::base::State *state)
{
    for (int i = 0; i < maxTries_; ++i)
    {
        internalSampler_->sampleUniform(state);
        double leftBound, rightBound;
        // get minimum time, when the state can be reached from the start
        double startBound = startMotion_->state->as<base::CompoundState>()
                                ->as<base::TimeStateSpace::StateType>(1)
                                ->position +
                            si_->getStateSpace()->as<base::SpaceTimeStateSpace>()->timeToCoverDistance(
                                state, startMotion_->state);
        // sample old batch
        if (sampleOldBatch_)
        {
            leftBound = startBound;
            // get maximum time, at which any goal can be reached from the state
            rightBound = std::numeric_limits<double>::min();
            for (auto goal : goalMotions_)
            {
                double t = goal->state->as<base::CompoundState>()
                               ->as<base::TimeStateSpace::StateType>(1)
                               ->position -
                           si_->getStateSpace()->as<base::SpaceTimeStateSpace>()->timeToCoverDistance(
                               goal->state, state);
                if (t > rightBound)
                {
                    rightBound = t;
                }
            }
        }
        // sample new batch
        else
        {
            // get maximum time, at which any goal from the new batch can be reached from the state
            rightBound = std::numeric_limits<double>::min();
            for (auto goal : newBatchGoalMotions_)
            {
                double t = goal->state->as<base::CompoundState>()
                               ->as<base::TimeStateSpace::StateType>(1)
                               ->position -
                           si_->getStateSpace()->as<base::SpaceTimeStateSpace>()->timeToCoverDistance(
                               goal->state, state);
                if (t > rightBound)
                {
                    rightBound = t;
                }
            }
            // get maximum time, at which any goal from the old batch can be reached from the state
            // only allow the left bound to be smaller than the right bound
            leftBound = std::numeric_limits<double>::min();
            for (auto goal : goalMotions_)
            {
                double t = goal->state->as<base::CompoundState>()
                               ->as<base::TimeStateSpace::StateType>(1)
                               ->position -
                           si_->getStateSpace()->as<base::SpaceTimeStateSpace>()->timeToCoverDistance(
                               goal->state, state);
                if (t > leftBound && t < rightBound)
                {
                    leftBound = t;
                }
            }
            leftBound = std::max(leftBound, startBound);
        }

        if (leftBound <= rightBound)
        {
            double time = rng_.uniformReal(leftBound, rightBound);
            state->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position = time;
            return true;
        }
    }
    return false;
}

bool ompl::base::ConditionalStateSampler::sampleNear(ompl::base::State *, const ompl::base::State *, double)
{
    throw ompl::Exception("ConditionalSampler::sampleNear", "not implemented");
}
