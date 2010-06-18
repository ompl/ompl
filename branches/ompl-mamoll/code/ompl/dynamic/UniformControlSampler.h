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

#ifndef OMPL_DYNAMIC_UNIFORM_CONTROL_SAMPLER_
#define OMPL_DYNAMIC_UNIFORM_CONTROL_SAMPLER_

#include "ompl/dynamic/ControlSampler.h"

namespace ompl
{
    
    namespace dynamic
    {
	
	class SpaceInformationControls;
	
	/** \brief A class that can perform sampling. Usually an instance of this class is needed
	 * for sampling controls */
	class UniformControlSampler : public ControlSampler
	{	    
	public:
	    UniformControlSampler(const SpaceInformationControls *si) : ControlSampler(si)
	    {
	    }	    
	    
	    virtual ~UniformControlSampler(void)
	    {
	    }
	    
	    /** \brief Sample a number of steps */
	    virtual unsigned int sampleStepCount(void);

	    /** \brief Sample a control */
	    virtual void sample(Control *ctrl);
	    
	    /** \brief Sample a control near another, within given bounds */
	    virtual void sampleNear(Control *ctrl, const Control *near, const double rho);
	    
	    /** \brief Sample a control near another, within given bounds */
	    virtual void sampleNear(Control *ctrl, const Control *near, const std::vector<double> &rho);
	    
	};
    }
}

#endif
