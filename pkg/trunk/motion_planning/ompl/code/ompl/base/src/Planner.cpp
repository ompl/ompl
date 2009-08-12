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

#include "ompl/base/Planner.h"
#include <ros/console.h>

ompl::base::PlannerType ompl::base::Planner::getType(void) const
{
    return m_type;
}

void ompl::base::Planner::setup(void)
{
    if (!m_si->isSetup())
	ROS_ERROR("Space information setup should have been called before planner setup was called");
    if (m_setup)
	ROS_ERROR("Planner setup called multiple times");		
    m_setup = true;
}

bool ompl::base::Planner::isTrivial(unsigned int *startID, double *distance) const
{
    Goal *goal = m_si->getGoal();
    
    if (!goal)
    {
	ROS_ERROR("Goal undefined");
	return false;
    }
    
    for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
    {
	State *start = m_si->getStartState(i);
	if (start && m_si->isValid(start) && m_si->satisfiesBounds(start))
	{
	    double dist;
	    if (goal->isSatisfied(start, &dist))
	    {
		if (startID)
		    *startID = i;
		if (distance)
		    *distance = dist;
		return true;
	    }	    
	}
	else
	{
	    ROS_ERROR("Initial state is in collision!");
	}
    }
    
    return false;    
}
