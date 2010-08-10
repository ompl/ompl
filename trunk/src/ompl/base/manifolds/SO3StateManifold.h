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

#ifndef OMPL_BASE_MANIFOLDS_SO3_STATE_MANIFOLD_
#define OMPL_BASE_MANIFOLDS_SO3_STATE_MANIFOLD_

#include "ompl/base/StateManifold.h"

namespace ompl
{
    namespace base
    {
	

	/** \brief Uniform sampler for the SO(3) manifold, using quaternion representation */
	class SO3StateUniformSampler : public StateSampler
	{
	public:
	    
	    SO3StateUniformSampler(const StateManifold *manifold) : StateSampler(manifold)
	    {
	    }
	    
	    virtual void sample(State *state);
	    virtual void sampleNear(State *state, const State *near, const double distance);
	};
	
	/** \brief A manifold representing SO(3). The internal
	    representation is done with quaternions. The distance
	    between states is the angle between quaternions and
	    interpolation is done with slerp. */
	class SO3StateManifold : public StateManifold
	{
	public:

	    
	    /** \brief The definition of a state in SO(3) represented as a unit quaternion
		
		\note The order of the elements matters in this
		definition for the SO3StateUniformSampler::sample()
		function. */
	    class StateType : public State
	    {
	    public:
		
		/** \brief Set the quaternion from axis-angle representation */
		void setAxisAngle(double ax, double ay, double az, double angle);
		
		/** \brief X component of quaternion vector */
		double x;

		/** \brief Y component of quaternion vector */
		double y;

		/** \brief Z component of quaternion vector */
		double z;

		/** \brief scalar component of quaternion */
		double w;
	    };
	
	    SO3StateManifold(void) : StateManifold()
	    {
	    }
	    
	    virtual ~SO3StateManifold(void)
	    {	
	    }

	    /** \brief Compute the norm of a state */
	    double norm(const StateType *state) const;
	    
	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const;
	    
	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(State *state) const;
	    	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const State *state) const;
	    
	    /** \brief Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const;
	    
	    /** \brief Computes distance to between two states */
	    virtual double distance(const State *state1, const State *state2) const;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const State *state1, const State *state2) const;

	    /** \brief Computes the state that lies at time t in [0, 1] on the
		segment that connects the current state to the
		destination state */
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

	    /** \brief Allocate an instance of a uniform state sampler for this space */
	    virtual StateSamplerPtr allocUniformStateSampler(void) const;
	    
	    /** \brief Allocate a state that can store a point in the described space */
	    virtual State* allocState(void) const;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(State *state) const;

	    /** \brief Print a state to screen */
	    virtual void printState(const State *state, std::ostream &out) const;
	    
	    /** \brief Print the settings for this manifold to a stream */
	    virtual void printSettings(std::ostream &out) const;
	    
	};
    }
}

#endif
