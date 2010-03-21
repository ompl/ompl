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

#ifndef OMPL_BASE_UNIFORM_STATE_SAMPLER_
#define OMPL_BASE_UNIFORM_STATE_SAMPLER_

#include "ompl/base/StateSampler.h"

namespace ompl
{
    
    namespace base
    {
	
	/** \brief A class that can perform sampling. Usually an instance of this class is needed
	 * for sampling states */
	class UniformStateSampler : public StateSampler
	{	    
	public:
	    UniformStateSampler(const SpaceInformation *si) : StateSampler(si)
	    {
	    }	    
	    
	    virtual ~UniformStateSampler(void)
	    {
	    }
	    
	    /** \brief Sample a state */
	    virtual void sample(base::State *state);
	    
	    /** \brief Sample a state near another, within given bounds */
	    virtual void sampleNear(base::State *state, const base::State *near, const double rho);
	    
	    /** \brief Sample a state near another, within given bounds */
	    virtual void sampleNear(base::State *state, const base::State *near, const std::vector<double> &rho);
	    
	};
    }
}

#endif
