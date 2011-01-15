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

#include "ompl/control/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <cassert>
#include <utility>
#include <limits>

void ompl::control::SpaceInformation::setup(void)
{
    base::SpaceInformation::setup();
    if (minSteps_ > maxSteps_)
        throw Exception("The minimum number of steps cannot be larger than the maximum number of steps");
    if (minSteps_ == 0 && maxSteps_ == 0)
    {
        minSteps_ = 1;
        maxSteps_ = 10;
        msg_.warn("Assuming propagation will always have between %d and %d steps", minSteps_, maxSteps_);
    }
    if (minSteps_ < 1)
        throw Exception("The minimum number of steps must be at least 1");

    if (stepSize_ < std::numeric_limits<double>::epsilon())
    {
        stepSize_ = getStateValidityCheckingResolution() * getMaximumExtent();
        if (stepSize_ < std::numeric_limits<double>::epsilon())
            throw Exception("The propagation step size must be larger than 0");
        msg_.warn("The propagation step size is assumed to be %f", stepSize_);
    }
    else
        // even if we need to do validation checking at a smaller resolution, we cannot go lower than the propagation step
        if (getStateValidityCheckingResolution() * getMaximumExtent() < stepSize_)
            msg_.warn("The state validity checking resolution is too small relative to the propagation step size. Resolution is %f%% (=%f), step size is %f. Ideally, the resolution should be an integer multiple of the step size. It however must be larger than the step size.", getStateValidityCheckingResolution() * 100.0, getStateValidityCheckingResolution() * getMaximumExtent(), stepSize_);

    controlManifold_->setup();
    if (controlManifold_->getDimension() <= 0)
        throw Exception("The dimension of the control manifold we plan in must be > 0");
}

void ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, base::State *result) const
{
    if (steps == 0)
    {
        if (result != state)
            copyState(result, state);
    }
    else
    {
        controlManifold_->propagate(state, control, stepSize_, result);
        for (unsigned int i = 1 ; i < steps ; ++i)
            controlManifold_->propagate(result, control, stepSize_, result);
    }
}

unsigned int ompl::control::SpaceInformation::propagateWhileValid(const base::State *state, const Control* control, unsigned int steps, base::State *result) const
{
    if (steps == 0)
    {
        if (result != state)
            copyState(result, state);
        return 0;
    }

    // perform the first step of propagation
    controlManifold_->propagate(state, control, stepSize_, result);

    // if we found a valid state after one step, we can go on
    if (isValid(result))
    {
        base::State *temp1 = result;
        base::State *temp2 = allocState();
        base::State *toDelete = temp2;
        unsigned int r = steps;

        // for the remaining number of steps
        for (unsigned int i = 1 ; i < steps ; ++i)
        {
            controlManifold_->propagate(temp1, control, stepSize_, temp2);
            if (isValid(temp2))
                std::swap(temp1, temp2);
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }

        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure result contains that information
        if (result != temp1)
            copyState(result, temp1);

        // free the temporary memory
        freeState(toDelete);

        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    else
    {
        if (result != state)
            copyState(result, state);
        return 0;
    }
}

void ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool alloc) const
{
    if (alloc)
    {
        result.resize(steps);
        for (unsigned int i = 0 ; i < result.size() ; ++i)
            result[i] = allocState();
    }
    else
    {
        if (result.empty())
            return;
        steps = std::min(steps, (unsigned int)result.size());
    }

    unsigned int st = 0;

    if (st < steps)
    {
        controlManifold_->propagate(state, control, stepSize_, result[st]);
        st++;

        while (st < steps)
        {
            controlManifold_->propagate(result[st-1], control, stepSize_, result[st]);
            st++;
        }
    }
}

unsigned int ompl::control::SpaceInformation::propagateWhileValid(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool alloc) const
{
    if (alloc)
        result.resize(steps);
    else
    {
        if (result.empty())
            return 0;
        steps = std::min(steps, (unsigned int)result.size());
    }

    unsigned int st = 0;

    if (st < steps)
    {
        if (alloc)
            result[st] = allocState();
        controlManifold_->propagate(state, control, stepSize_, result[st]);
        st++;

        if (isValid(result[st-1]))
        {
            while (st < steps)
            {
                if (alloc)
                    result[st] = allocState();
                controlManifold_->propagate(result[st-1], control, stepSize_, result[st]);
                st++;
                if (!isValid(result[st-1]))
                {
                    if (alloc)
                    {
                        freeState(result[st-1]);
                        result.resize(st);
                    }
                    break;
                }
            }
        }
        else
        {
            if (alloc)
            {
                freeState(result[st-1]);
                result.resize(st);
            }
        }
    }

    return st;
}

void ompl::control::SpaceInformation::printSettings(std::ostream &out) const
{
    base::SpaceInformation::printSettings(out);
    out << "  - control manifold:" << std::endl;
    controlManifold_->printSettings(out);
    out << "  - propagation step size: " << stepSize_ << std::endl;
    out << "  - propagation duration: [" << minSteps_ << ", " << maxSteps_ << "]" << std::endl;
}
