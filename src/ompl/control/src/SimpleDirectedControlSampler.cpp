/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include "ompl/control/SimpleDirectedControlSampler.h"
#include "ompl/control/SpaceInformation.h"

ompl::control::SimpleDirectedControlSampler::SimpleDirectedControlSampler(const SpaceInformation *si, unsigned int k)
  : DirectedControlSampler(si), cs_(si->allocControlSampler()), numControlSamples_(k)
{
}

ompl::control::SimpleDirectedControlSampler::~SimpleDirectedControlSampler() = default;

unsigned int ompl::control::SimpleDirectedControlSampler::sampleTo(Control *control, const base::State *source,
                                                                   base::State *dest)
{
    return getBestControl(control, source, dest, nullptr);
}

unsigned int ompl::control::SimpleDirectedControlSampler::sampleTo(Control *control, const Control *previous,
                                                                   const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest, previous);
}

unsigned int ompl::control::SimpleDirectedControlSampler::getBestControl(Control *control, const base::State *source,
                                                                         base::State *dest, const Control *previous)
{
    // Sample the first control
    if (previous != nullptr)
        cs_->sampleNext(control, previous, source);
    else
        cs_->sample(control, source);

    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
    // Propagate the first control, and find how far it is from the target state
    base::State *bestState = si_->allocState();
    steps = si_->propagateWhileValid(source, control, steps, bestState);

    if (numControlSamples_ > 1)
    {
        Control *tempControl = si_->allocControl();
        base::State *tempState = si_->allocState();
        double bestDistance = si_->distance(bestState, dest);

        // Sample k-1 more controls, and save the control that gets closest to target
        for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
            unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
            if (previous != nullptr)
                cs_->sampleNext(tempControl, previous, source);
            else
                cs_->sample(tempControl, source);

            sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
            double tempDistance = si_->distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
                si_->copyState(bestState, tempState);
                si_->copyControl(control, tempControl);
                bestDistance = tempDistance;
                steps = sampleSteps;
            }
        }

        si_->freeState(tempState);
        si_->freeControl(tempControl);
    }

    si_->copyState(dest, bestState);
    si_->freeState(bestState);

    return steps;
}
