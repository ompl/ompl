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

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include "ompl/base/Goal.h"
#include "ompl/base/Path.h"
#include "ompl/base/util/output.h"
#include <cstdlib>
#include <vector>

/** Main namespace */
namespace ompl
{
    
    namespace base
    {
	
	/** The base class for space information. This contains all the
	    information about the space planning is done in */
	class SpaceInformation
	{
	public:
	    
	    /** Constructor */
	    SpaceInformation(void)
	    {
		m_goal = NULL;
		m_setup = false;
	    }
	    
	    /** Destructor */
	    virtual ~SpaceInformation(void)
	    {
	    }

	    /** Add a start state */
	    void addStartState(State *state)
	    {
		m_startStates.push_back(state);
	    }
	    
	    /** Clear all start states (memory is freed) */
	    void clearStartStates(void)
	    {
		for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
		    delete m_startStates[i];
		m_startStates.clear();
	    }
	    
	    /** Clear all start states but do not free memory */
	    void forgetStartStates(void)
	    {
		m_startStates.clear();
	    }
	    
	    /** Returns the number of start states */
	    unsigned int getStartStateCount(void) const
	    {
		return m_startStates.size();
	    }
	    
	    /** Returns a specific start state */
	    State* getStartState(unsigned int index) const
	    {
		return m_startStates[index];
	    }

	    /** Set the goal. The memory for a previous goal is freed. */
	    void setGoal(Goal *goal)
	    {
		if (m_goal)
		    delete m_goal;
		m_goal = goal;
	    }
	    
	    /** Clear the goal. Memory is freed. */
	    void clearGoal(void)
	    {
		if (m_goal)
		    delete m_goal;
		m_goal = NULL;
	    }
	    
	    /** Return the current goal */
	    Goal* getGoal(void) const
	    {
		return m_goal;
	    }
	    
	    /** Clear the goal, but do not free its memory */
	    void forgetGoal(void)
	    {
		m_goal = NULL;
	    }
	    
	    /************************************************************/
	    /* Utility functions                                        */
	    /************************************************************/
	    
	    /** Perform additional setup tasks (run once, before use) */
	    virtual void setup(void)
	    {
		m_setup = true;
	    }

	protected:
	    
	    bool                    m_setup;
	    msg::Interface          m_msg;
	    
	    std::vector<State*>     m_startStates;
	    Goal                   *m_goal;
	    
	};
	
	
    }
    
}
    
#endif
