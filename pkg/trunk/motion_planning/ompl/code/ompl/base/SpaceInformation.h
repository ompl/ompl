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

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/General.h"
#include <vector>

namespace ompl
{

    ForwardClassDeclaration(SpaceInformation);
    
    class SpaceInformation
    {
    public:
	SpaceInformation(void)
	{
	    m_goal = NULL;	    
	}
	
	virtual ~SpaceInformation(void)
	{
	}
	

	/************************************************************/
	/* Paths                                                    */
	/************************************************************/

	ForwardClassDeclaration(Path);
	
	class Path
	{
	public:
	    
	    Path(SpaceInformation_t si)
	    {
		m_si = si;
	    }
	    
	    virtual ~Path(void)
	    {
	    }

	protected:
	    
	    SpaceInformation_t m_si;
	};
	
	    

	/************************************************************/
	/* Goals                                                    */
	/************************************************************/
	
	ForwardClassDeclaration(Goal);
	
	class Goal
	{
	public:

	    Goal(SpaceInformation_t si)
	    {
		m_si   = si;
		m_path = NULL;
		m_difference = -1.0;
		m_approximate = false;
	    }
	    
	    virtual ~Goal(void)
	    {
		if (m_path)
		    delete m_path;
	    }
	    
	    bool isAchieved(void) const
	    {
		return m_path != NULL;
	    }

	    Path_t getSolutionPath(void) const
	    {
		return m_path;
	    }
	    
	    void setSolutionPath(Path_t path, bool approximate = false)
	    {
		m_path = path;
		m_approximate = approximate;
	    }
	    
	    double getDifference(void) const
	    {
		return m_difference;
	    }
	    
	    void setDifference(double difference)
	    {
		m_difference = difference;
	    }
	    
	    bool isApproximate(void) const
	    {
		return m_approximate;
	    }
	    
	protected:
	    
	    /** solution path, if found */
	    Path_t             m_path;
	    
	    /** the space information for this goal */
	    SpaceInformation_t m_si;
	    
	    /** the achieved difference between the found solution and the desired goal */
	    double             m_difference;

	    /** true if goal was not achieved, but an approximate solution was found */
	    bool               m_approximate;
	    
	};
	
	void setGoal(Goal_t goal)
	{
	    if (m_goal)
		delete m_goal;
	    m_goal = goal;
	}
	
	void clearGoal(bool free = true)
	{
	    if (free && m_goal)
		delete m_goal;
	    m_goal = NULL;
	}

	Goal_t getGoal(void) const
	{
	    return m_goal;
	}
	
	
	/************************************************************/
	/* States                                                   */
	/************************************************************/

	ForwardClassDeclaration(State);
	
	class State
	{
	public:
	    virtual ~State(void)
	    {
	    }
	};
	
	void addStartState(State_t state)
	{
	    m_startStates.push_back(state);
	}
	
	void clearStartStates(bool free = true)
	{
	    if (free)
		for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
		    delete m_startStates[i];
	    m_startStates.clear();
	}
	
	unsigned int getStartStateCount(void) const
	{
	    return m_startStates.size();
	}

	State_t getStartState(unsigned int index) const
	{
	    return m_startStates[index];
	}
	
    protected:
	
	Goal_t               m_goal;
	std::vector<State_t> m_startStates;
	
    };
    
	

}

#endif
