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

/** Main namespace */
namespace ompl
{

    /** Forward class declaration */
    ForwardClassDeclaration(Planner);
    
    /** Base class for a planner */
    class Planner
    {
	
    public:
	
	/** Constructor */
	Planner(SpaceInformation_t si)
	{
	    m_si    = si;
	    m_type  = 0;
	    m_setup = false;
	}
	
	/** Destructor */
	virtual ~Planner(void)
	{
	}
	
	/** Function that can solve the motion planning problem */
	virtual bool solve(double solveTime) = 0;

	/** Clear all internal datastructures */
	virtual void clear(void) = 0;
	
	/** Perform extra configuration steps, if needed */
	virtual void setup(void)
	{
	    m_setup = true;
	}

	/** Return the type of the motion planner. This is useful if
	    the planner wants to advertise what type of problems it
	    can solve */
	int getType(void)
	{
	    return m_type;
	}
	
	/** A problem is trivial if the given starting state already
	    in the goal region, so we need no motion planning. startID
	    will be set to the index of the starting state that
	    satisfies the goal. The distance to the goal can
	    optionally be returned as well. */
	virtual bool isTrivial(unsigned int *startID = NULL, double *distance = NULL);
	
    protected:
	
	SpaceInformation_t m_si;
	int                m_type;	
	bool               m_setup;
	msg::Interface     m_msg;
    };    

}


#endif
