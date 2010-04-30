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

#ifndef OMPL_BASE_MOTION_PLANNER_
#define OMPL_BASE_MOTION_PLANNER_

#include "ompl/base/General.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Console.h"
#include "ompl/util/Time.h"

namespace ompl
{

    namespace base
    {
	
	enum PlannerType
	    {
		/** \brief This value should not be set */
		PLAN_UNKNOWN        = 0,

		/** \brief This bit is set if planning to goal states (ompl::base::GoalState) is possible */
		PLAN_TO_GOAL_STATE  = 1,

		/** \brief This bit is set if planning to sampleable goal regions (ompl::base::GoalSampleableRegion) is possible */
		PLAN_TO_GOAL_SAMPLEABLE_REGION = 2 | PLAN_TO_GOAL_STATE,

		/** \brief This bit is set if planning to goal regions (ompl::base::GoalRegion) is possible */
		PLAN_TO_GOAL_REGION = 4 | PLAN_TO_GOAL_SAMPLEABLE_REGION,
		
		/** \brief This bit is set if planning to generic goal regions (ompl::base::Goal) is possible */
		PLAN_TO_GOAL_ANY    = 32768 | PLAN_TO_GOAL_REGION
	    };
	
	/** \brief Base class for a planner */
	class Planner
	{
	    
	public:
	    
	    /** \brief Constructor */
	    Planner(SpaceInformation *si);
	    
	    /** \brief Destructor */
	    virtual ~Planner(void)
	    {
	    }
	    
	    /** \brief Function that can solve the motion planning problem */
	    virtual bool solve(double solveTime) = 0;
	    
	    /** \brief Clear all internal datastructures */
	    virtual void clear(void) = 0;
	    
	    /** \brief Get states in current exploration datastructure */
	    virtual void getStates(std::vector<const base::State*> &states) const = 0;
	    
	    /** \brief A problem is trivial if the given starting state already
		in the goal region, so we need no motion planning. startID
		will be set to the index of the starting state that
		satisfies the goal. The distance to the goal can
		optionally be returned as well. */
	    virtual bool isTrivial(unsigned int *startID = NULL, double *distance = NULL) const;
	    
	    /** \brief Return the type of the motion planner. This is useful if
		the planner wants to advertise what type of problems it
		can solve */
	    PlannerType getType(void) const;
	    
	    /** \brief Perform extra configuration steps, if needed */
	    virtual void setup(void);
	    
	protected:
	    
	    SpaceInformation *m_si;
	    PlannerType       m_type;	
	    bool              m_setup;
	    msg::Interface    m_msg;
	};    
    }
}


#endif
