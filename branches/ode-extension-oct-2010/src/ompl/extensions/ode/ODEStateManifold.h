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

#ifndef OMPL_EXTENSION_ODE_STATE_MANIFOLD_
#define OMPL_EXTENSION_ODE_STATE_MANIFOLD_

#include "ompl/base/StateManifold.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/SO3StateManifold.h"
#include "ompl/extensions/ode/ODEEnvironment.h"

namespace ompl
{
    namespace control
    {
	
	class ODEStateManifold : public base::CompoundStateManifold
	{
	public:
	    static const int STATE_COLLISION_TRUE     = 1;
	    static const int STATE_COLLISION_FALSE    = -1;
	    static const int STATE_COLLISION_UNKNOWN  = 0;
	    
	    class StateType : public base::CompoundStateManifold::StateType
	    {
	    public:
		StateType(void) : base::CompoundStateManifold::StateType(), collision(STATE_COLLISION_UNKNOWN)
		{
		}
		
		const double* getBodyPosition(unsigned int body) const
		{
		    return as<base::RealVectorStateManifold::StateType>(body * 4)->values;
		}
		
		double* getBodyPosition(unsigned int body)
		{
		    return as<base::RealVectorStateManifold::StateType>(body * 4)->values;
		}
		
		const base::SO3StateManifold::StateType& getBodyRotation(unsigned int body) const
		{
		    return *as<base::SO3StateManifold::StateType>(body * 4 + 3);
		}
		
		base::SO3StateManifold::StateType& getBodyRotation(unsigned int body)
		{
		    return *as<base::SO3StateManifold::StateType>(body * 4 + 3);
		}
		
		const double* getBodyLinearVelocity(unsigned int body) const
		{
		    return as<base::RealVectorStateManifold::StateType>(body * 4 + 1)->values;
		}
		
		double* getBodyLinearVelocity(unsigned int body)
		{
		    return as<base::RealVectorStateManifold::StateType>(body * 4 + 1)->values;
		}
		
		const double* getBodyAngularVelocity(unsigned int body) const
		{
		    return as<base::RealVectorStateManifold::StateType>(body * 4 + 2)->values;
		}
		
		double* getBodyAngularVelocity(unsigned int body)
		{
		    return as<base::RealVectorStateManifold::StateType>(body * 4 + 2)->values;
		}
		
		/** \brief Flag indicating whether this state is known
		    to be in collision or not. Initially, this flag is
		    STATE_COLLISION_UNKNOWN for all states. This flag
		    is equal to STATE_COLLISION_TRUE only if some
		    bodies in the simulation collide AND
		    isValidCollision() returns false. If the state is
		    in collision, then it is invalid. If there is no
		    collision, or isValidCollision() returns true, the
		    value of the flag is STATE_COLLISION_FALSE. This
		    does not directly imply the state is valid. The
		    purpose of the flag is to avoid unnecessary
		    collision checking calls. */
		mutable int collision;
		
	    };
	    
	    ODEStateManifold(const ODEEnvironment &env);
	    
	    virtual ~ODEStateManifold(void)
	    {
	    }
	    
	    const ODEEnvironment& getEnvironment(void) const
	    {
		return env_;
	    }
	    
	    void setVolumeBounds(const base::RealVectorBounds &bounds);

	    void setLinearVelocityBounds(const base::RealVectorBounds &bounds);

	    void setAngularVelocityBounds(const base::RealVectorBounds &bounds);

	    /** \brief Read the parameters of the ODE bodies and store
		them in \e state. */
	    virtual void readState(base::State *state) const;
	    
	    /** \brief Set the parameters of the ODE bodies to be the
		ones read from \e state.  The code will technically work if
		this function is called from multiple threads
		simultaneously, but the results are unpredictable. */
	    virtual void writeState(const base::State *state) const;
	    
	    /** \brief This is a convenience function provided for
		optimization purposes. It checks whether a state
		satisfies its bounds. Typically, in the process of
		simulation the rotations remain valid (very slightly
		out of bounds), so there is no point in updating or
		checking them. This function checks all other bounds
		(position, linear and agular velocities) */
	    bool satisfiesBoundsExceptRotation(const StateType *state) const;
	    
	    virtual base::State* allocState(void) const;
	    virtual void freeState(base::State *state) const;
	    virtual void copyState(base::State *destination, const base::State *source) const;

	    virtual void evaluateCollision(const base::State *source) const;

	protected:
	    
	    const ODEEnvironment &env_;
	};
    }
}


#endif
