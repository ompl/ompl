/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Beck Chen, Mark Moll */

#ifndef DEMOS_KOULES_DIRECTEDCONTROLSAMPLER_
#define DEMOS_KOULES_DIRECTEDCONTROLSAMPLER_

#include "KoulesControlSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/Goal.h>

// Directed control sampler
class KoulesDirectedControlSampler : public ompl::control::DirectedControlSampler
{
public:
    // The KoulesDirectedControlSampler attempts to steer the system towards
    // a desired state. It takes as additional arguments a goal pointer and a
    // a flag called propagateMax. The goal pointer is needed to stop the
    // motion when the goal is reached before the desired state. The
    // propagateMax flag indicates that the motion is always extended
    // upto the maximum control duration (rather than a randomly sampled limit
    // between the min and max control duration)
    KoulesDirectedControlSampler(const ompl::control::SpaceInformation *si,
        const ompl::base::GoalPtr &goal, bool propagateMax)
        : DirectedControlSampler(si), cs_(si->getControlSpace().get()),
        goal_(goal), statePropagator_(si->getStatePropagator()),
        propagateMax_(propagateMax)
    {
    }
    // This sampleTo implementation contains a modified version of the method
    // ompl::control::SpaceInformation::propagateWhileValid, with the key difference
    // that sampleTo also terminates when the goal is reached.
    virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest);

    virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::control::Control * /* previous */,
        const ompl::base::State *source, ompl::base::State *dest)
    {
        return sampleTo(control, source, dest);
    }
protected:
    KoulesControlSampler                     cs_;
    ompl::RNG                                rng_;
    const ompl::base::GoalPtr                goal_;
    const ompl::control::StatePropagatorPtr  statePropagator_;
    bool                                     propagateMax_;
};

#endif
