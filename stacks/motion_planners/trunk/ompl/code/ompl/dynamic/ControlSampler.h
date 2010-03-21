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

#ifndef OMPL_DYNAMIC_CONTROL_SAMPLER_
#define OMPL_DYNAMIC_CONTROL_SAMPLER_

#include "ompl/datastructures/RandomNumbers.h"
#include "ompl/dynamic/Control.h"
#include <vector>

namespace ompl
{
    namespace dynamic
    {
	
	class SpaceInformationControls;

	/** \brief Abstract definition of a control sampler */
	class ControlSampler
	{	    
	public:
	    ControlSampler(const SpaceInformationControls *si);
	    
	    virtual ~ControlSampler(void)
	    {
	    }
	    
	    /** \brief Sample a number of steps */
	    virtual unsigned int sampleStepCount(void) = 0;
	    
	    /** \brief Sample a control */
	    virtual void sample(Control *ctrl) = 0;
	    
	    /** \brief Sample a control near another, within given bounds */
	    virtual void sampleNear(Control *ctrl, const Control *near, const double rho) = 0;
	    
	    /** \brief Sample a control near another, within given bounds */
	    virtual void sampleNear(Control *ctrl, const Control *near, const std::vector<double> &rho) = 0;
	    
	    /** \brief Return a reference to the random number generator used */
	    RNG& getRNG(void)
	    {
		return m_rng;
	    }
	    
	protected:
	    
	    const SpaceInformationControls *m_si;
	    RNG                             m_rng;
	    unsigned int                    m_minControlDuration;
	    unsigned int                    m_maxControlDuration;
	};
	
	/** \brief Simple class to make instantiating control samplers easier */
	class ControlSamplerInstance
	{
	public:

	    ControlSamplerInstance(const SpaceInformationControls *si);
	    ~ControlSamplerInstance(void);
	    
	    /** \brief Allow easy access the functions of the contained sampler */
	    ControlSampler* operator->(void)
	    {
		return m_sampler;
	    }
	    
	private:
	    
	    ControlSampler *m_sampler;
	};
	
    }    
}


#endif
