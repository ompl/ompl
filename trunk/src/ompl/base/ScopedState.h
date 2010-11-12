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

#ifndef OMPL_BASE_SCOPED_STATE_
#define OMPL_BASE_SCOPED_STATE_

#include "ompl/base/SpaceInformation.h"
#include <boost/concept_check.hpp>
#include <iostream>

namespace ompl
{
    namespace base
    {
	
	/** \brief Definition of a scoped state. 

	    This class allocates a state of a desired type using the
	    allocation mechanism of the manifold the state is part
	    of. The state is then freed when the instance goes out of
	    scope using the corresponding free mechanism. */
	template<class T = StateManifold>
	class ScopedState
	{
	    /** \brief Make sure the type we are allocating is indeed from a manifold */
	    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
	    
	    /** \brief Make sure the type we are allocating is indeed a state */
	    BOOST_CONCEPT_ASSERT((boost::Convertible<typename T::StateType*, State*>));
	    
	public:
	    
	    /** \brief The type of the contained state */
	    typedef typename T::StateType StateType;
	    
	    /** \brief Given the space that we are working with,
		allocate a state from the corresponding
		manifold. Throw an exception if the desired type to
		cast this state into does not match the type of states
		allocated. */
	    explicit
	    ScopedState(const SpaceInformationPtr &si) : manifold_(si->getStateManifold())
	    {	
		State *s = manifold_->allocState();

		// ideally, this should be a dynamic_cast and we
		// should throw an exception in case of
		// failure. However, RTTI may not be available across
		// shared library boundaries, so we do not use it
		state_ = static_cast<StateType*>(s);
	    }
	    
	    /** \brief Given the manifold that we are working with,
		allocate a state. Throw an exception if the desired
		type to cast this state into does not match the type
		of states allocated. */
	    explicit
	    ScopedState(const StateManifoldPtr &manifold) : manifold_(manifold)
	    {
		State *s = manifold_->allocState();
		
		// ideally, this should be a dynamic_cast and we
		// should throw an exception in case of
		// failure. However, RTTI may not be available across
		// shared library boundaries, so we do not use it
		state_ = static_cast<StateType*>(s);
	    }

	    /** \brief Copy constructor */
	    ScopedState(const ScopedState<T> &other) : manifold_(other.getManifold())
	    { 
		State *s = manifold_->allocState();
		state_ = static_cast<StateType*>(s);
		manifold_->copyState(s, static_cast<const State*>(other.get()));
	    }

	    /** \brief Copy constructor that allows instantiation from states of other type */
	    template<class O>
	    ScopedState(const ScopedState<O> &other) : manifold_(other.getManifold())
	    { 
		BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateManifold*>));
		BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

		// ideally, we should use a dynamic_cast and throw an
		// exception in case other.get() does not cast to
		// const StateType*. However, RTTI may not be
		// available across shared library boundaries, so we
		// do not use it
		
		State *s = manifold_->allocState();
		state_ = static_cast<StateType*>(s);
		manifold_->copyState(s, static_cast<const State*>(other.get()));
	    }


	    /** \brief Free the memory of the internally allocated state */
	    ~ScopedState(void)
	    {	
		manifold_->freeState(state_);
	    }	    

	    /** \brief Get the manifold that the state corresponds to */
	    const StateManifoldPtr& getManifold(void) const
	    {
		return manifold_;
	    }
	    
	    /** \brief Assignment operator */
	    ScopedState<T>& operator=(const ScopedState<T> &other)
	    {
		if (&other != this)
		{
		    manifold_->freeState(state_);
		    manifold_ = other.getManifold();
		    
		    State *s = manifold_->allocState();
		    state_ = static_cast<StateType*>(s);
		    manifold_->copyState(s, static_cast<const State*>(other.get()));
		}
		return *this;
	    }

	    /** \brief Assignment operator */
	    ScopedState<T>& operator=(const State *other)
	    {
		if (other != static_cast<State*>(state_))
		{
		    // ideally, we should use a dynamic_cast and throw an
		    // exception in case other does not cast to
		    // const StateType*. However, RTTI may not be
		    // available across shared library boundaries, so we
		    // do not use it
		    
		    manifold_->copyState(static_cast<State*>(state_), other);
		}
		return *this;
	    }

	    /** \brief Assignment operator */
	    ScopedState<T>& operator=(const State &other)
	    {
		if (&other != static_cast<State*>(state_))
		{
		    // ideally, we should use a dynamic_cast and throw an
		    // exception in case &other does not cast to
		    // const StateType*. However, RTTI may not be
		    // available across shared library boundaries, so we
		    // do not use it
		    
		    manifold_->copyState(static_cast<State*>(state_), &other);
		}
		return *this;
	    }

	    /** \brief Assignment operator that allows conversion of states */
	    template<class O>
	    ScopedState<T>& operator=(const ScopedState<O> &other)
	    {
		BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateManifold*>));
		BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));
		
		// ideally, we should use a dynamic_cast and throw an
		// exception in case other.get() does not cast to
		// const StateType*. However, RTTI may not be
		// available across shared library boundaries, so we
		// do not use it
		
		if (reinterpret_cast<const void*>(&other) != reinterpret_cast<const void*>(this))
		{
		    manifold_->freeState(state_);
		    manifold_ = other.getManifold();
		    
		    State *s = manifold_->allocState();
		    state_ = static_cast<StateType*>(s);
		    manifold_->copyState(s, static_cast<const State*>(other.get()));
		}
		return *this;
	    }
	    
	    /** \brief Checks equality of two states */
	    template<class O>
	    bool operator==(const ScopedState<O> &other) const
	    {
		BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateManifold*>));	
		BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

		// ideally, we should use a dynamic_cast and throw an
		// exception in case other.get() does not cast to
		// const StateType*. However, RTTI may not be
		// available across shared library boundaries, so we
		// do not use it
		
		return manifold_->equalStates(static_cast<const State*>(state_), static_cast<const State*>(other.get()));
	    }

	    /** \brief Checks equality of two states */	    
	    template<class O>
	    bool operator!=(const ScopedState<O> &other) const
	    {
		return !(*this == other);
	    }
	    
	    /** \brief Set this state to a random value (uniform) */
	    void random(void)
	    {
		if (!sampler_)
		    sampler_ = manifold_->allocStateSampler();
		sampler_->sampleUniform(state_);
	    }

	    /** \brief Print this state to a stream */
	    void print(std::ostream &out = std::cout) const
	    {
		manifold_->printState(state_, out);
	    }

	    /** \brief De-references to the contained state */
	    StateType& operator*(void) const
	    {
		return *state_;
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    StateType* operator->(void) const
	    {
		return state_;
	    }
	    
	    /** \brief Returns a pointer to the contained state */
	    StateType* get(void) const
	    {
		return state_;
	    }

	    /** \brief Returns a pointer to the contained state */
	    StateType* operator()(void) const
	    {
		return state_;
	    }

	private:
	    
	    StateManifoldPtr         manifold_;
	    ManifoldStateSamplerPtr  sampler_;
	    StateType               *state_;
	};

	
	/** \brief Overload stream output operator */
	inline
	std::ostream& operator<<(std::ostream &out, const ScopedState<> &state)
	{
	    state.print(out);
	    return out;
	}

    }
}

#endif
