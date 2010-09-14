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

#ifndef OMPL_BASE_STATE_VALIDITY_CHECKER_
#define OMPL_BASE_STATE_VALIDITY_CHECKER_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    
    namespace base
    {

	ClassForward(SpaceInformation);

	/** \brief Forward declaration of ompl::base::StateValidityChecker */
	ClassForward(StateValidityChecker);

	/** \class ompl::base::StateValidityCheckerPtr
	    \brief A boost shared pointer wrapper for ompl::base::StateValidityChecker */

	/** \brief Abstract definition for a class checking the
	    validity of states. The implementation of this class must
	    be thread safe. */
	class StateValidityChecker
	{
	public:

	    /** \brief Constructor */
	    StateValidityChecker(SpaceInformation* si) : si_(si)
	    {
	    }	

	    /** \brief Constructor */
	    StateValidityChecker(const SpaceInformationPtr &si) : si_(si.get())
	    {
	    }
	    
	    virtual ~StateValidityChecker(void)
	    {
	    }
	    
	    /** \brief Return true if the state is valid. Usually, this means at least collision checking. If it is
		possible that ompl::base::StateManifold::interpolate() or ompl::control::ControlManifold::propagate() return states that
		are outside of bounds, this function should also make a call to ompl::base::SpaceInformation::satisfiesBounds(). */
	    virtual bool isValid(const State *state) const = 0;
	    
	protected:
	    
	    /** \brief The instance of space information this state validity checker operates on */
	    SpaceInformation *si_;
	    
	};
	
	/** \brief The simplest state validity checker: all states are valid */
	class AllValidStateValidityChecker : public StateValidityChecker
	{
	public:

	    /** \brief Constructor */
	    AllValidStateValidityChecker(SpaceInformation* si) : StateValidityChecker(si)
	    {
	    }	

	    /** \brief Constructor */
	    AllValidStateValidityChecker(const SpaceInformationPtr &si) : StateValidityChecker(si)
	    {
	    }
	    
	    virtual bool isValid(const State * /* state */ ) const
	    {
		return true;
	    }
	};
	
    }
}

#endif
