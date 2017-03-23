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

#include "KoulesDirectedControlSampler.h"
#include "KoulesStateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

unsigned int KoulesDirectedControlSampler::sampleTo(oc::Control *control, const ob::State *source, ob::State *dest)
{
    const double* dstPos = dest->as<KoulesStateSpace::StateType>()->values;
    double stepSize = si_->getPropagationStepSize();
    unsigned int steps = propagateMax_ ? si_->getMaxControlDuration() :
        cs_.sampleStepCount(si_->getMinControlDuration(), si_->getMaxControlDuration());

    cs_.steer(control, source, dstPos[0], dstPos[1]);
    // perform the first step of propagation
    statePropagator_->propagate(source, control, stepSize, dest);
    // if we reached the goal, we're done
    if (goal_->isSatisfied(dest))
        return 1;
    // if we found a valid state after one step, we can go on
    if (si_->isValid(dest))
    {
        ob::State *temp1 = dest, *temp2 = si_->allocState(), *toDelete = temp2;
        unsigned int r = steps;
        for (unsigned int i = 1 ; i < steps ; ++i)
        {
            statePropagator_->propagate(temp1, control, stepSize, temp2);
            if (goal_->isSatisfied(dest))
            {
                si_->copyState(dest, temp2);
                si_->freeState(toDelete);
                return i + 1;
            }
            if (si_->isValid(temp2))
                std::swap(temp1, temp2);
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }
        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure dest contains that information
        if (dest != temp1)
            si_->copyState(dest, temp1);
        si_->freeState(toDelete);
        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    else
    {
        if (dest != source)
            si_->copyState(dest, source);
        return 0;
    }
}
