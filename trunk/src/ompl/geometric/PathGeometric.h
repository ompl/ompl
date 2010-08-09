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

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PATH_GEOMETRIC_
#define OMPL_GEOMETRIC_PATH_GEOMETRIC_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include <vector>

namespace ompl
{

    /** \brief This namespace contains code that is specific to planning under geometric constraints */
    namespace geometric
    {

	/** \brief Definition of a geometric path.

	    This is the type of path computed by geometric planners. */
	class PathGeometric : public base::Path
	{
	public:
	    
	    /** \brief Construct a path instance for a given space information */
	    PathGeometric(const base::SpaceInformationPtr &si) : base::Path(si)
	    {
	    }

	    /** \brief Copy constructor */
	    PathGeometric(const PathGeometric &path);
	    
	    virtual ~PathGeometric(void)
	    {
		freeMemory();
	    }
	    
	    /** \brief Assignment operator */
	    PathGeometric& operator=(const PathGeometric& other);
	    
	    /** \brief Compute the length of a geometric path */
	    virtual double length(void) const;

	    /** \brief Check if the path is valid */
	    virtual bool check(void) const;	

	    /** \brief Print the path to a stream */
	    virtual void print(std::ostream &out) const;

	    /** \brief Insert states in a path, at the collision checking resolution */
	    void interpolate(double factor = 1.0);
	    
	    /** \brief The list of states that make up the path */
	    std::vector<base::State*> states;
	    
	protected:
	    
	    /** \brief Free the memory corresponding to the states on this path */
	    void freeMemory(void);

	    /** \brief Copy data to this path from another path instance */
	    void copyFrom(const PathGeometric& other);
	};
	
    }
}

#endif
