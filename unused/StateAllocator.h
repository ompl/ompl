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

#ifndef OMPL_BASE_STATE_ALLOCATOR_
#define OMPL_BASE_STATE_ALLOCATOR_

#include "ompl/base/StateManifold.h"
#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {
	
	/** \brief Definition of an state allocator. This reuses
	    memory for states and frees them only when the instance is
	    destroyed. */
	class StateAllocator : private boost::noncopyable
	{
	public:

	    /** \brief Constructor */
	    StateAllocator(const StateManifoldPtr &manifold);
	    
	    ~StateAllocator(void);
	    
	    /** \brief Allocate a state from the specified manifold */
	    State* allocState(void) const;
	    
	    /** \brief Free the memory of the allocated state */
	    void freeState(State *state) const;
	    
	    /** \brief Clear all the allocated memory */
	    void clear(void);
	    
	    /** \brief Return the number of pre-allocated states */
	    std::size_t sizeAvailable(void) const;

	    /** \brief Return the number of allocated states that are in use */
	    std::size_t sizeInUse(void) const;

	    /** \brief Return the number of allocated states */
            std::size_t sizeTotal(void) const;
            
	private:
            
            StateManifoldPtr manifold_;
            void            *data_;
	};

    }
}

#endif
