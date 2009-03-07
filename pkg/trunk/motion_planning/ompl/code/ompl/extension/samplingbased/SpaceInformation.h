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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_SPACE_INFORMATION_
#define OMPL_EXTENSION_SAMPLINGBASED_SPACE_INFORMATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateDistanceEvaluator.h"
#include "ompl/base/StateValidityChecker.h"

#include "ompl/extension/samplingbased/State.h"
#include "ompl/extension/samplingbased/Goal.h"

#include "ompl/base/util/random_utils.h"

#include <vector>
#include <iostream>

/** Main namespace */
namespace ompl
{
    
    namespace sb
    {
	
	/** Space information useful for kinematic planning */
	class SpaceInformation : public base::SpaceInformation
	{
	public:
	    
	    /** Constructor; setup() needs to be called as well, before use */
	    SpaceInformation(void) : base::SpaceInformation()
	    {
		m_stateDistanceEvaluator = NULL;
		m_stateValidityChecker = NULL;
		m_stateDimension = 0;
	    }
	    
	    /** Destructor */
	    virtual ~SpaceInformation(void)
	    {
	    }

	    /** Set the instance of the distance evaluator to use. This is
		only needed by some planning algorithms. No memory freeing is performed. */
	    void setStateDistanceEvaluator(base::StateDistanceEvaluator *sde)
	    {
		m_stateDistanceEvaluator = sde;
	    }
	    
	    /** Return the instance of the used state distance evaluator */
	    base::StateDistanceEvaluator* getStateDistanceEvaluator(void) const
	    {
		return m_stateDistanceEvaluator;
	    }	
	    
	    /** Set the instance of the validity checker to use. No memory freeing is performed. */
	    void setStateValidityChecker(base::StateValidityChecker *svc)
	    {
		m_stateValidityChecker = svc;
	    }
	    
	    /** Return the instance of the used state validity checker */
	    base::StateValidityChecker* getStateValidityChecker(void) const
	    {
		return m_stateValidityChecker;
	    }
	    
	    /** Return the dimension of the state space */
	    unsigned int getStateDimension(void) const
	    {
		return m_stateDimension;
	    }
	    
	    /** Get information about a component of the state space */
	    const StateComponent& getStateComponent(unsigned int index) const
	    {
		return m_stateComponent[index];
	    }
	    
	    /** Check if a given state is valid or not */
	    bool isValid(const State *state) const
	    {
		return (*m_stateValidityChecker)(static_cast<const base::State*>(state));
	    }
	    
	    /** Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const;
	    
	    /** Check if a state is inside the bounding box */
	    bool satisfiesBounds(const State *s) const;
	    
	    /** Compute the distance between two states */
	    double distance(const State *s1, const State *s2) const
	    {
		return (*m_stateDistanceEvaluator)(static_cast<const base::State*>(s1), static_cast<const base::State*>(s2));
	    }
	    
	    /** Print a state to a stream */
	    void printState(const State *state, std::ostream &out = std::cout) const;

	    /** Print information about the current instance of the state space */
	    virtual void printSettings(std::ostream &out = std::cout) const;
	    
	    /** Perform additional tasks to finish the initialization of
		the space information */
	    virtual void setup(void);
	    
	protected:
	    
	    unsigned int                   m_stateDimension;
	    std::vector<StateComponent>    m_stateComponent;
	    base::StateValidityChecker    *m_stateValidityChecker;
	    base::StateDistanceEvaluator  *m_stateDistanceEvaluator;
	    
	};
	
    }
}

#endif
