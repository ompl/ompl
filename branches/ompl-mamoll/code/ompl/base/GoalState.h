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

#ifndef OMPL_BASE_GOAL_STATE_
#define OMPL_BASE_GOAL_STATE_

#include "ompl/base/GoalSampleableRegion.h"

namespace ompl
{
    
    namespace base
    {
	
	/** \brief Definition of a goal state */
	class GoalState : public GoalSampleableRegion
	{
	public:
	    
	    GoalState(const SpaceInformation *si) : GoalSampleableRegion(si), state(NULL)
	    {
	    }
	    
	    virtual ~GoalState(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    /** \brief Sample a state in the goal region */
	    virtual void sampleGoal(base::State *st) const;
	    
	    /** \brief Return the maximum number of samples that can be asked for before repeating */
	    virtual unsigned int maxSampleCount(void) const;
	    
	    /** \brief Compute the distance to the goal (heuristic) */
	    virtual double distanceGoal(const base::State *st) const;	    
	    
	    /** \brief Print information about the goal data structure
		to a stream */
	    virtual void print(std::ostream &out = std::cout) const;
	    
	    /** \brief The goal state */
	    base::State *state;
	};

    }
}

#endif
