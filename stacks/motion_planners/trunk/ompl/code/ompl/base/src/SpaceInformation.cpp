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

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/GoalState.h"
#include "ompl/base/UniformStateSampler.h"
#include <angles/angles.h>
#include <ros/console.h>
#include <sstream>
#include <cstring>
#include <cassert>

namespace ompl
{
    namespace base
    {
	static StateSampler* allocUniformStateSampler(const SpaceInformation *si)
	{
	    return new UniformStateSampler(si);
	}
    }
}

void ompl::base::SpaceInformation::copyState(State *destination, const State *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
}

void ompl::base::SpaceInformation::setup(void)
{
    if (m_setup)
	ROS_ERROR("Space information setup called multiple times");
    
    if (m_stateDimension <= 0)
	ROS_FATAL("State space dimension must be > 0");
    
    if (m_stateComponent.size() != m_stateDimension)
	ROS_FATAL("State component specification does not agree with state dimension");
    
    if (!m_stateSamplerAllocator)
	m_stateSamplerAllocator = boost::bind(allocUniformStateSampler, _1);
    
    m_setup = true;
}

bool ompl::base::SpaceInformation::isSetup(void) const
{
    return m_setup;
}

void ompl::base::SpaceInformation::printState(const State *state, std::ostream &out) const
{
    if (state)
    {
	for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    out << state->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

bool ompl::base::SpaceInformation::satisfiesBounds(const State *s) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (s->values[i] - STATE_EPSILON > m_stateComponent[i].maxValue ||
	    s->values[i] + STATE_EPSILON < m_stateComponent[i].minValue)
	    return false;
    return true;
}

bool ompl::base::SpaceInformation::fixInvalidInputStates(const std::vector<double> &rhoStart, const std::vector<double> &rhoGoal, unsigned int attempts)
{
    assert(rhoStart.size() == rhoGoal.size() && rhoStart.size() == m_stateDimension);
    
    bool result = true;
    
    // fix start states
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
    {
	base::State *st = m_startStates[i];
	if (st)
	{
	    bool b = satisfiesBounds(st);
	    bool v = false;
	    if (b)
	    {
		v = isValid(st);
		if (!v)
		    ROS_DEBUG("Initial state is not valid");
	    }
	    else
		ROS_DEBUG("Initial state is not within space bounds");
	    
	    if (!b || !v)
	    {
		std::stringstream ss;
		printState(st, ss);
		ss << " within margins [ ";
		for (unsigned int j = 0 ; j < rhoStart.size() ; ++j)
		    ss << rhoStart[j] << " ";
		ss << "]";		
		ROS_DEBUG("Attempting to fix initial state %s", ss.str().c_str());
		base::State temp(m_stateDimension);
		if (searchValidNearby(&temp, st, rhoStart, attempts))
		    copyState(st, &temp);
		else
		{
		    ROS_WARN("Unable to fix start state %u", i);
		    result = false;
		}
	    }
	}
    }
    
    // fix goal state
    base::GoalState *goal = dynamic_cast<base::GoalState*>(m_goal);
    if (goal)
    {
	base::State *st = goal->state;
	if (st)
	{
	    bool b = satisfiesBounds(st);
	    bool v = false;
	    if (b)
	    {
		v = isValid(st);
		if (!v)
		    ROS_DEBUG("Goal state is not valid");
	    }
	    else
		ROS_DEBUG("Goal state is not within space bounds");
	    
	    if (!b || !v)
	    {
		
		std::stringstream ss;
		printState(st, ss);
		ss << " within margins [ ";
		for (unsigned int i = 0 ; i < rhoGoal.size() ; ++i)
		    ss << rhoGoal[i] << " ";
		ss << "]";
		ROS_DEBUG("Attempting to fix goal state %s", ss.str().c_str());
		base::State temp(m_stateDimension);
		if (searchValidNearby(&temp, st, rhoGoal, attempts))
		    copyState(st, &temp);
		else
		{
		    ROS_WARN("Unable to fix goal state");
		    result = false;
		}
	    }
	}
    }
    
    return result;    
}

void ompl::base::SpaceInformation::enforceBounds(base::State *state) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    {
	if (state->values[i] > m_stateComponent[i].maxValue)
	    state->values[i] = m_stateComponent[i].maxValue;
	else
	    if (state->values[i] < m_stateComponent[i].minValue)
		state->values[i] = m_stateComponent[i].minValue;
    }
}

bool ompl::base::SpaceInformation::searchValidNearby(base::State *state, const base::State *near, const std::vector<double> &rho, unsigned int attempts) const
{
    assert(near != state);
    
    copyState(state, near);
    
    // fix bounds, if needed
    if (!satisfiesBounds(state))
	enforceBounds(state);
    
    bool result = isValid(state);
    
    if (!result)
    {
	// try to find a valid state nearby
	StateSamplerInstance ss(this);
	base::State          temp(m_stateDimension);
	copyState(&temp, state);	
	for (unsigned int i = 0 ; i < attempts && !result ; ++i)
	{
	    ss->sampleNear(state, &temp, rho);
	    result = isValid(state);
	}
    }
    
    return result;
}

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "Kinematic state space settings:" << std::endl;
    out << "  - dimension = " << m_stateDimension << std::endl;
    out << "  - start states:" << std::endl;
    for (unsigned int i = 0 ; i < getStartStateCount() ; ++i)
	printState(dynamic_cast<const State*>(getStartState(i)), out);
    if (m_goal)
	m_goal->print(out);
    else
	out << "  - goal = NULL" << std::endl;
    out << "  - bounding box:" << std::endl;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	out << "[" << m_stateComponent[i].minValue << ", " <<  m_stateComponent[i].maxValue << "](" << m_stateComponent[i].resolution << ") ";
    out << std::endl;
}
