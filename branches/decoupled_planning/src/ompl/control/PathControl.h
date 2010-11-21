/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_PATH_CONTROL_
#define OMPL_CONTROL_PATH_CONTROL_

#include "ompl/control/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/geometric/PathGeometric.h"
#include <vector>

namespace ompl
{
    namespace control
    {
	
	/** \brief Definition of a control path.

	 This is the type of path produced when planning with
	 differential constraints. */
	class PathControl : public base::Path
	{
	public:
	   
	    /** \brief Constructor */
	    PathControl(const base::SpaceInformationPtr &si);
	    
	    /** \brief Copy constructor */
	    PathControl(const PathControl &path);
	    
	    virtual ~PathControl(void)
	    {
		freeMemory();
	    }

	    /** \brief Assignment operator */
	    PathControl& operator=(const PathControl& other);

	    /** \brief The path length (sum of control durations) */
	    virtual double length(void) const;

	    /** \brief Check if the path is valid */
	    virtual bool check(void) const;

	    /** \brief Print the path to a stream */
	    virtual void print(std::ostream &out) const;
	    
	    /** \brief Make the path such that all controls are applied for a single time step (computes intermediate states) */
	    void interpolate(void);
	    
	    /** \brief Convert this path into a geometric path (interpolation is performed and then states are copied) */
	    geometric::PathGeometric asGeometric(void) const;
	    
	    /** \brief The list of states that make up the path */
	    std::vector<base::State*>   states;

	    /** \brief The control applied at each state. This array contains one element less than the list of states */
	    std::vector<Control*>       controls;

	    /** \brief The duration of the control applied at each state. This array contains one element less than the list of states */
	    std::vector<double>         controlDurations;
	    
	protected:
	    
	    /** \brief Free the memory allocated by the path */
	    void freeMemory(void);

	    /** \brief Copy the content of a path to this one */
	    void copyFrom(const PathControl& other);
	    
	};
	
    }
}

#endif
