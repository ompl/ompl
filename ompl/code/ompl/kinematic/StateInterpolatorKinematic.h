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

#ifndef OMPL_KINEMATIC_STATE_INTERPOLATOR_KINEMATIC_
#define OMPL_KINEMATIC_STATE_INTERPOLATOR_KINEMATIC_

#include "ompl/base/SpaceInformation.h"
#include <vector>

namespace ompl
{
    namespace kinematic
    {
	
	/** \brief Abstract definition of a state interpolator */
	class StateInterpolatorKinematic
	{
	public:
	    
	    StateInterpolatorKinematic(const base::SpaceInformation *si) : m_si(si)
	    {
	    }
	    
	    virtual ~StateInterpolatorKinematic(void)
	    {
	    }
	    
	    /** \brief Check if a motion is valid.

		Given two states, this function must check whether the
		path segment computed according to the interpolation
		rules is valid. If the user desires (one or both of
		lastValidState and lastValidTime are not NULL), this
		function must compute the last valid state and the
		time of that state. The time is defined to be in the
		interval [0, 1], where s1 is at time 0 and s2 is at
		time 1. */
	    virtual bool checkMotion(const base::State *s1, const base::State *s2,
				     base::State *lastValidState = NULL, double *lastValidTime = NULL) const = 0;
	    
	    /** \brief This function must compute the vector of states
		that lie on the path segment between s1 and s2, at some
		discretiation. 

		The discretization is controlled by factor. If factor
		= 1, a default discretization (as the implementer
		chooses) is to be selected. If the factor is higher,
		fewer states will be included in the
		discretization. This function should be able to resize
		the vector of states and allocate the memory for the
		filled in states if the alloc flag is set to true. If
		the flag is set to false, the function must only fill
		in at most the first states.size() elements of the
		discretized path segment. In this case, it need not
		allocate memory for the individual states. If the
		endpoints flag is set to true, the discretization
		should include the states s1 and s2, as
		endpoints. Otherwise, it should not. The number of
		filled in states is returned. */
	    virtual unsigned int getStates(const base::State *s1, const base::State *s2,
					   std::vector<base::State*> &states,
					   double factor, bool endpoints, bool alloc) const = 0;
	    
	protected:
	    
	    const base::SpaceInformation *m_si;
	    
	};
    }
}


#endif

