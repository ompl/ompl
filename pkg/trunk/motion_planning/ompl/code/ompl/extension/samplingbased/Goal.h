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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_GOAL_
#define OMPL_EXTENSION_SAMPLINGBASED_GOAL_

#include "ompl/base/Goal.h"
#include <cstdlib>

namespace ompl
{
    
    namespace sb
    {
	class SpaceInformation;
	
	/** Definition of a goal region */
	class GoalRegion : public base::Goal
	{
	public:
	    
	    GoalRegion(SpaceInformation *si);
	    
	    virtual ~GoalRegion(void)
	    {
	    }
	    
	    /** Decide whether a given state is part of the goal
		region. Returns true if the distance to goal is less
		than the threshold */
	    virtual bool isSatisfied(const base::State *s, double *distance = NULL) const;
	    
	    /** Compute the distance to the goal (heuristic) */
	    virtual double distanceGoal(const base::State *s) const = 0;
	    
	    /** Print information about the goal data structure to the
		screen */
	    virtual void print(std::ostream &out = std::cout) const;
	    
	    /** The maximum distance that is allowed to the goal */
	    double threshold;
	};
	
	/** Definition of a goal state */
	class GoalState : public GoalRegion
	{
	public:
	    
	    GoalState(SpaceInformation *si);
	    
	    virtual ~GoalState(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    /** Compute the distance to the goal (heuristic) */
	    virtual double distanceGoal(const base::State *s) const;	    
	    
	    /** Print information about the goal data structure to the
		screen */
	    virtual void print(std::ostream &out = std::cout) const;
	    
	    /** The goal state */
	    base::State *state;
	};
	
    }
    
}

#endif
