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

/* \author Ioan Sucan */

#include "ompl/dynamic/SpaceInformationControlsIntegrator.h"
#include <ros/console.h>
#include <cassert>
#include <algorithm>
#include <queue>

void ompl::dynamic::SpaceInformationControlsIntegrator::setup(void)
{
    if (!m_stateValidityChecker)
	ROS_FATAL("No state validity checker defined");
    
    if (!m_stateForwardPropagator)
	ROS_FATAL("No forward propagation routine defined");    

    SpaceInformationControls::setup();
}

unsigned int ompl::dynamic::SpaceInformationControlsIntegrator::getMotionStates(const base::State *begin, const Control *ctrl, unsigned int steps, std::vector<base::State*> &states, bool alloc) const
{
    if (alloc)
    {
	states.resize(steps + 1);
	for (unsigned int i = 0 ; i <= steps ; ++i)
	    states[i] = new base::State(m_stateDimension);
    }
    else
    {	
	if (states.empty())
	    return 0;
	steps = std::min(steps, (unsigned int)states.size() - 1);
    }
    
    unsigned int st = 1;
    copyState(states.front(), begin);
    
    while (st <= steps)
    {
	(*m_stateForwardPropagator)(states[st - 1], ctrl, m_resolution, states[st]);
	st++;
    }
    return st;
}

bool ompl::dynamic::SpaceInformationControlsIntegrator::checkStatesIncremental(const std::vector<base::State*> &states, unsigned int count, unsigned int *firstInvalidStateIndex) const
{
    assert(states.size() >= count);
    for (unsigned int i = 0 ; i < count ; ++i)
	if (!isValid(states[i]))
	{
	    if (firstInvalidStateIndex)
		*firstInvalidStateIndex = i;
	    return false;
	}
    return true;
}

bool ompl::dynamic::SpaceInformationControlsIntegrator::checkStatesSubdivision(const std::vector<base::State*> &states, unsigned int count) const
{ 
    assert(states.size() >= count);
    if (count > 0)
    {
	if (count > 1)
	{
	    if (!isValid(states.front()))
		return false;
	    if (!isValid(states[count - 1]))
		return false;
	    
	    // we have 2 or more states, and the first and last states are valid
	    
	    if (count > 2)
	    {
		std::queue< std::pair<int, int> > pos;
		pos.push(std::make_pair(0, count - 1));
	    
		while (!pos.empty())
		{
		    std::pair<int, int> x = pos.front();
		    
		    int mid = (x.first + x.second) / 2;
		    if (!isValid(states[mid]))
			return false;

		    if (x.first < mid - 1)
			pos.push(std::make_pair(x.first, mid));
		    if (x.second > mid + 1)
			pos.push(std::make_pair(mid, x.second));
		}
	    }
	}
	else
	    return isValid(states.front());
    }
    return true;
}
