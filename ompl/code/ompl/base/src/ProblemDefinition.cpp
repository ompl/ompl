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

#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/GoalState.h"

#include <sstream>
#include <cassert>

bool ompl::base::ProblemDefinition::hasStartState(const State *state, unsigned int *startIndex)
{
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
	if (m_si->equalStates(state, m_startStates[i]))
	{
	    if (startIndex)
		*startIndex = i;
	    return true;
	}
    return false;
}

bool ompl::base::ProblemDefinition::fixInvalidInputStates(const std::vector<double> &rhoStart, const std::vector<double> &rhoGoal, unsigned int attempts)
{
    assert(rhoStart.size() == rhoGoal.size() && rhoStart.size() == m_si->getStateDimension());
    
    bool result = true;
    
    // fix start states
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
    {
	base::State *st = m_startStates[i];
	if (st)
	{
	    bool b = m_si->satisfiesBounds(st);
	    bool v = false;
	    if (b)
	    {
		v = m_si->isValid(st);
		if (!v)
		    m_msg.debug("Initial state is not valid");
	    }
	    else
		m_msg.debug("Initial state is not within space bounds");
	    
	    if (!b || !v)
	    {
		std::stringstream ss;
		m_si->printState(st, ss);
		ss << " within margins [ ";
		for (unsigned int j = 0 ; j < rhoStart.size() ; ++j)
		    ss << rhoStart[j] << " ";
		ss << "]";		
		m_msg.debug("Attempting to fix initial state %s", ss.str().c_str());
		base::State temp(m_si->getStateDimension());
		if (m_si->searchValidNearby(&temp, st, rhoStart, attempts))
		    m_si->copyState(st, &temp);
		else
		{
		    m_msg.warn("Unable to fix start state %u", i);
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
	    bool b = m_si->satisfiesBounds(st);
	    bool v = false;
	    if (b)
	    {
		v = m_si->isValid(st);
		if (!v)
		    m_msg.debug("Goal state is not valid");
	    }
	    else
		m_msg.debug("Goal state is not within space bounds");
	    
	    if (!b || !v)
	    {
		
		std::stringstream ss;
		m_si->printState(st, ss);
		ss << " within margins [ ";
		for (unsigned int i = 0 ; i < rhoGoal.size() ; ++i)
		    ss << rhoGoal[i] << " ";
		ss << "]";
		m_msg.debug("Attempting to fix goal state %s", ss.str().c_str());
		base::State temp(m_si->getStateDimension());
		if (m_si->searchValidNearby(&temp, st, rhoGoal, attempts))
		    m_si->copyState(st, &temp);
		else
		{
		    m_msg.warn("Unable to fix goal state");
		    result = false;
		}
	    }
	}
    }
    
    return result;    
}

bool ompl::base::ProblemDefinition::isTrivial(unsigned int *startIndex, double *distance) const
{
    if (!m_goal)
    {
	m_msg.error("Goal undefined");
	return false;
    }
    
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
    {
	const State *start = m_startStates[i];
	if (start && m_si->isValid(start) && m_si->satisfiesBounds(start))
	{
	    double dist;
	    if (m_goal->isSatisfied(start, start, &dist))
	    {
		if (startIndex)
		    *startIndex = i;
		if (distance)
		    *distance = dist;
		return true;
	    }	    
	}
	else
	{
	    m_msg.error("Initial state is in collision!");
	}
    }
    
    return false;    
}

void ompl::base::ProblemDefinition::print(std::ostream &out) const
{
    out << "Start states:" << std::endl;
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
	m_si->printState(m_startStates[i], out);
    if (m_goal)
	m_goal->print(out);
    else
	out << "Goal = NULL" << std::endl;
}
