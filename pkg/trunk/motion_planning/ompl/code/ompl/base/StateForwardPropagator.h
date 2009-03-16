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

#ifndef OMPL_BASE_STATE_FORWARD_PROPAGATOR_
#define OMPL_BASE_STATE_FORWARD_PROPAGATOR_

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include "ompl/base/Control.h"
#include <cstdlib>

namespace ompl
{
    
    namespace base
    {
	
	/** Abstract definition for a class perfoming forward integration. The () operator must be defined.
	 *  This definition assumes no collision checking is performed when propagating forward */
	class StateForwardPropagator
	{
	public:

	    /** Destructor */
	    virtual ~StateForwardPropagator(void)
	    {
	    }
	    
	    /** Propagate the system forward in time, given a starting state, a control and a duration. The result is a state
		and some (potentially) some flags (such as collision found, if collision checking was performed) */
	    virtual void operator()(const State *begin, const Control *ctrl, unsigned int steps, double resolution, State *end) const = 0;
	};
	
	
	/** Abstract definition for a class perfoming forward integration. The () operator must be defined. 
	 *  This definition assumes collision checking (contacts are modelled) is also performed when propagating forward */
	class StateForwardPropagatorWithContacts
	{
	public:
	    
	    struct Options
	    {
	        Options(void) : stop_at_contact(true), test_contact_only(false)
		{		    
		}
		
	        Options(bool sac, bool tco) : stop_at_contact(sac), test_contact_only(tco)
		{
		}
		
		bool stop_at_contact;
		bool test_contact_only;
	    };
	    
	    struct Result
	    {
	        Result(void) : first_contact_step(-1), end(NULL)
		{
		}
		
	        Result(State *e) : first_contact_step(-1), end(e)
		{
		}
		
		int    first_contact_step; // if a collision found, this is the step at which that collision was found. if no collision, -1; 
		State *end;                // last reached state
	    };
	    
	    
	    /** Destructor */
	    virtual ~StateForwardPropagatorWithContacts(void)
	    {
	    }
	    
	    /** Propagate the system forward in time, given a starting state, a control and a duration. The result is a state
		and some (potentially) some flags (such as collision found, if collision checking was performed) */
	    virtual void operator()(const State *begin, const Control *ctrl, unsigned int steps, double resolution,
				    Options &options, Result &result) const = 0;
	};
	
    }
}

#endif
