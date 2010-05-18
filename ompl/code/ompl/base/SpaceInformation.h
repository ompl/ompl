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

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include "ompl/base/StateDistanceEvaluator.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/StateSampler.h"

#include "ompl/util/Console.h"

#include <cstdlib>
#include <vector>
#include <iostream>

#include <boost/function.hpp>
#include <boost/bind.hpp>

/** \brief Main namespace */
namespace ompl
{
    
    /** \brief The epsilon for checking whether a state is within bounds or equal to another state */
    
    const double STATE_EPSILON = 1e-12;
    
    /** \brief This namespace contains sampling based planning
	routines shared by both planning under geometric constraints
	(kinematic) and planning under differential constraints
	(dynamic) */
    namespace base
    {
	
	/** \brief The base class for space information. This contains
	    all the information about the space planning is done in.
	    setup() needs to be called as well, before use */
	class SpaceInformation
	{
	public:
	    
	    /** \brief Constructor */
	    SpaceInformation(void)
	    {
		m_setup = false;
		m_stateDistanceEvaluator = NULL;
		m_stateValidityChecker = NULL;
		m_stateSamplerAllocator = NULL;
		m_stateDimension = 0;
	    }
	    
	    /** \brief Destructor */
	    virtual ~SpaceInformation(void)
	    {
	    }
	    
	    /** \brief Set the pointer to a function that can allocate
		state samplers.  We do not set a specific instance
		since parallel planners will use multiple instances in
		order to benefit from different random seeds. */
	    void setStateSamplerAllocator(const boost::function<StateSampler*(const SpaceInformation*)> &sampler)
	    {
		m_stateSamplerAllocator = sampler;
	    }
	    
	    
	    /** \brief Allocate a new state sampler. Releasing the memory is not handled. */
	    StateSampler* allocNewStateSampler(void) const
	    {
		return m_stateSamplerAllocator(this);
	    }
	    
	    /** \brief Set the instance of the distance evaluator to
		use. This is only needed by some planning
		algorithms. No memory freeing is performed. Parallel
		implementations of algorithms assume the distance
		evaluator is thread safe. */
	    void setStateDistanceEvaluator(StateDistanceEvaluator *sde)
	    {
		m_stateDistanceEvaluator = sde;
	    }

	    /** \brief Return the instance of the used state distance evaluator */
	    StateDistanceEvaluator* getStateDistanceEvaluator(void) const
	    {
		return m_stateDistanceEvaluator;
	    }	
	    
	    /** \brief Set the instance of the validity checker to
		use. No memory freeing is performed. Parallel
		implementations of planners assume this validity
		checker is thread safe. */
	    void setStateValidityChecker(StateValidityChecker *svc)
	    {
		m_stateValidityChecker = svc;
	    }
	    
	    /** \brief Return the instance of the used state validity checker */
	    StateValidityChecker* getStateValidityChecker(void) const
	    {
		return m_stateValidityChecker;
	    }
	    
	    /** \brief Specify the state to use in the space information */
	    void setStateComponents(const std::vector<StateComponent> &stateSpec)
	    {
		m_stateComponent = stateSpec;
		m_stateDimension = stateSpec.size();
	    }

	    /** \brief Specify the state to use in the space information */
	    void setStateComponent(const StateComponent &stateSpec, unsigned int index)
	    {
		m_stateComponent[index] = stateSpec;
	    }
	    
	    /** \brief Return the dimension of the state space */
	    unsigned int getStateDimension(void) const
	    {
		return m_stateDimension;
	    }
	    
	    /** \brief Get information about a all the state space components */
	    const std::vector<StateComponent>& getStateComponents(void) const
	    {
		return m_stateComponent;
	    }
	    
	    /** \brief Get information about a component of the state space */
	    const StateComponent& getStateComponent(unsigned int index) const
	    {
		return m_stateComponent[index];
	    }
	    
	    /** \brief Check if a given state is valid or not */
	    bool isValid(const State *state) const
	    {
		return (*m_stateValidityChecker)(state);
	    }
	    
	    /** \brief Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const;

	    /** \brief Check if two states are the same */
	    virtual bool equalStates(const State *a, const State *b) const;
	    
	    /** \brief Check if a state is inside the bounding box */
	    bool satisfiesBounds(const State *s) const;
	    
	    /** \brief Compute the distance between two states */
	    double distance(const State *s1, const State *s2) const
	    {
		return (*m_stateDistanceEvaluator)(s1, s2);
	    }

	    /** \brief Bring the state within the bounds of the state space */
	    void enforceBounds(base::State *state) const;
	    
	    /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
	     *  The two passed state pointers must point to different states. Returns true on success.  */
	    bool searchValidNearby(base::State *state, const base::State *near, const std::vector<double> &rho, unsigned int attempts) const;

	    /************************************************************/
	    /* Utility functions                                        */
	    /************************************************************/
	    
	    /** \brief Print a state to a stream */
	    void printState(const State *state, std::ostream &out = std::cout) const;

	    /** \brief Print information about the current instance of the state space */
	    virtual void printSettings(std::ostream &out = std::cout) const;
	    
	    /** \brief Perform additional setup tasks (run once, before use) */
	    virtual void setup(void);
	    
	    /** \brief Return true if setup was called */
	    bool isSetup(void) const;
	    
	protected:
	    	    

	    unsigned int                 m_stateDimension;
	    std::vector<StateComponent>  m_stateComponent;
	    StateValidityChecker        *m_stateValidityChecker;
	    StateDistanceEvaluator      *m_stateDistanceEvaluator;
	    
	    boost::function<StateSampler*(const SpaceInformation*)>
	                                 m_stateSamplerAllocator;
	    
	    bool                         m_setup;

	    msg::Interface               m_msg;
	};
	
    }
    
}
    
#endif
