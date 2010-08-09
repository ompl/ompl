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
	stepSize_ = resolution_;
	msg_.warn("The propagation step size is assumed to be the same as the state validity checking resolution: %f", stepSize_);
    }

    controlManifold_->setup();    
    if (controlManifold_->getDimension() <= 0)
	throw Exception("The dimension of the control manifold we plan in must be > 0");
}

void ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, base::State *result) const
{
    if (steps == 0)
	copyState(result, state);
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
	copyState(result, state);
	return 0;
    }
    
    // perform the first step of propagation
    PropagationResult pr = controlManifold_->propagate(state, control, stepSize_, result);
    
    // if the propagator we use cannot tell state validity, we ignore the PropagationResult
    if (pr == PROPAGATION_START_UNKNOWN)
    {
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
	    copyState(result, state);
	    return 0;
	}
    }
    // if it looks like the employed propagator CAN tell state validity
    else
    {
	// it is assumed the starting state is valid
	assert(pr == PROPAGATION_START_VALID);
	
	base::State *temp1 = allocState();
	base::State *temp2 = result;
	base::State *temp3 = allocState();
	unsigned int r = steps;
	
	// for the remaining number of steps
	for (unsigned int i = 1 ; i < steps ; ++i)
	{
	    pr = controlManifold_->propagate(temp2, control, stepSize_, temp3);
	    
	    // compute the validity of temp2
	    bool valid = pr == PROPAGATION_START_UNKNOWN ? isValid(temp2) : pr == PROPAGATION_START_VALID;

	    if (valid)
	    {
		std::swap(temp1, temp2);
		std::swap(temp2, temp3);
	    }
	    else
	    {
		// we found temp2 to be invalid; the valid state before temp2 is temp1;
		// if however we are at the first step, temp1 is not yet initialized
		// and the correct value of result should be the start state
		if (temp1 != result)
		    copyState(result, i == 1 ? state : temp1);
		r = i - 1;
		break;
	    }
	}
	
	// if we finished the for-loop without finding an invalid state, 
	// we need to check the final state for validity (this is temp2, because of the swap that gets executed)
	if (r == steps)
	{
	    if (isValid(temp2))
	    {	
		if (result != temp2)
		    copyState(result, temp2);
	    }
	    else
	    {
		r--;
		if (result != temp1)
		    copyState(result, temp1);
	    }
	}
	
	// free the temporary memory
	if (temp1 != result)
	    freeState(temp1);
	if (temp2 != result)
	    freeState(temp2);
	if (temp3 != result)
	    freeState(temp3);
	
	return r;
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
	PropagationResult pr = controlManifold_->propagate(state, control, stepSize_, result[st]);
	st++;
	
	if (pr == PROPAGATION_START_UNKNOWN)
	{
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
	else
	{
	    // we know pr = PROPAGATION_START_VALID at this point
	    while (st < steps)
	    {
		if (alloc)
		    result[st] = allocState();
		
		PropagationResult pr = controlManifold_->propagate(result[st-1], control, stepSize_, result[st]);
		bool valid = pr == PROPAGATION_START_UNKNOWN ? isValid(result[st-1]) : pr == PROPAGATION_START_VALID;
		
		if (!valid)
		{
		    // the state at st-2 is the last valid one
		    if (alloc)
		    {
			freeState(result[st]);
			freeState(result[st-1]);
			result.resize(st - 1);
		    }
		    st--;
		    break;
		}
		st++;
	    }
	    if (st == steps)
	    {
		if (!isValid(result[st-1]))
		{
		    st--;
		    if (alloc)
		    {
			freeState(result[st]);
			result.resize(st);
		    }
		}
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

