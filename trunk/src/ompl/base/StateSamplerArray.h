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

#ifndef OMPL_BASE_STATE_SAMPLER_ARRAY_
#define OMPL_BASE_STATE_SAMPLER_ARRAY_

#include "ompl/base/StateSampler.h"
#include "ompl/util/ClassForward.h"
#include <vector>
   
namespace ompl
{
    namespace base
    {

	ClassForward(SpaceInformation);
	
	/** \brief Class to ease the creation of a set of samplers. This is especially useful for multi-threaded planners. */
	class StateSamplerArray 
	{
	public:
	    
	    /** \brief Constructor */
	    StateSamplerArray(const SpaceInformationPtr &si) : si_(si)
	    {
	    }
	    
	    ~StateSamplerArray(void)
	    {
	    }
	    
	    /** \brief Access operator for a specific sampler. For
		performance reasons, the bounds are not checked. */
	    StateSampler* operator[](std::size_t index)
	    {
		return samplers_[index].get();
	    }

	    /** \brief Create or release some state samplers */
	    void resize(std::size_t count);
	    
	    /** \brief Get the count of samplers currently available */
	    std::size_t size(void) const
	    {
		return samplers_.size();
	    }
	    
	private:
	    
	    std::vector<StateSamplerPtr> samplers_;
	    SpaceInformationPtr          si_;
	};
    }
}

#endif
