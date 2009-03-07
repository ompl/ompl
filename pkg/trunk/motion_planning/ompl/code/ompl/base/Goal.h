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

#ifndef OMPL_BASE_GOAL_
#define OMPL_BASE_GOAL_

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include "ompl/base/Path.h"
#include <iostream>
#include <cstdlib>

namespace ompl
{
    namespace base
    {
	
	class SpaceInformation;
	
	/** Abstract definition of goals. Will contain solutions, if found */
	class Goal
	{
	public:
	    
	    /** Constructor. The goal must always know the space information it is part of */
	    Goal(SpaceInformation *si)
	    {
		m_si   = si;
		m_path = NULL;
		m_difference = -1.0;
		m_approximate = false;
	    }
	    
	    /** Destructor. Clears the solution as well */
	    virtual ~Goal(void)
	    {
		if (m_path)
		    delete m_path;
	    }
	    
	    /** Return true if the state statisfies the goal
	     *  constraints.  If the state does not satisfy the
	     *  constraints, set the distance of how far the state
	     *  is from the goal. */
	    virtual bool isSatisfied(const State *s, double *distance) const = 0;
	    
	    /** Returns the space information this goal is part of */
	    SpaceInformation* getSpaceInformation(void) const
	    {
		return m_si;
	    }
	    
	    /** Returns true if a solution path has been found (could be approximate) */
	    bool isAchieved(void) const
	    {
		return m_path != NULL;
	    }
	    
	    /** Return the found solution path */
	    Path* getSolutionPath(void) const
	    {
		return m_path;
	    }
	    
	    /** Forget the solution path. Memory is not freed. This is
		useful when the user wants to keep the solution path
		but wants to clear the goal. The user takes
		responsibilty to free the memory for the solution
		path. */
	    void forgetSolutionPath(void)
	    {
		m_path = NULL;
	    }	    
	    
	    /** Update the solution path. If a previous solution path exists, it is deleted. */
	    void setSolutionPath(Path *path, bool approximate = false)
	    {
		if (m_path)
		    delete m_path;
		m_path = path;
		m_approximate = approximate;
	    }
	    
	    /** If a difference between the desired solution and the
		solution found is computed by the planner, this functions
		returns it */
	    double getDifference(void) const
	    {
		return m_difference;
	    }
	    
	    /** Set the difference between the found solution path and
		the desired solution path */
	    void setDifference(double difference)
	    {
		m_difference = difference;
	    }
	    
	    /** Return true if the found solution is approximate (does not actually reach the desired goal,
		but hopefully is closer to it) */
	    bool isApproximate(void) const
	    {
		return m_approximate;
	    }
	    
	    /** Print information about the goal */
	    virtual void print(std::ostream &out = std::cout) const
	    {
		out << "Goal memory address " << reinterpret_cast<const void*>(this) << std::endl;
	    }
	    
	protected:
	    
	    /** solution path, if found */
	    Path              *m_path;
	    
	    /** the space information for this goal */
	    SpaceInformation  *m_si;
	    
	    /** the achieved difference between the found solution and the desired goal */
	    double             m_difference;
	    
	    /** true if goal was not achieved, but an approximate solution was found */
	    bool               m_approximate;
	    
	};
	
	
    }
}

#endif
