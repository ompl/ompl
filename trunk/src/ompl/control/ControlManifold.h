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

#ifndef OMPL_CONTROL_CONTROL_MANIFOLD_
#define OMPL_CONTROL_CONTROL_MANIFOLD_

#include "ompl/base/StateManifold.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <vector>

namespace ompl
{

    namespace control
    {

	/** \brief Forward declaration of ompl::control::ControlManifold */	
	ClassForward(ControlManifold);
	
	/** \class ompl::control::ControlManifoldPtr
	    \brief A boost shared pointer wrapper for ompl::control::ControlManifold */

	/** \brief A function that achieves state propagation.*/
	typedef boost::function4<void, const base::State*, const Control*, const double, base::State*> StatePropagationFn;
	
	/** \brief A manifold representing the space of applicable controls */
	class ControlManifold : private boost::noncopyable
	{
	public:
	    
	    /** \brief Construct a control manifold, given the state manifold */
	    ControlManifold(const base::StateManifoldPtr &stateManifold) : stateManifold_(stateManifold)
	    {
		name_ = "control[" + stateManifold_->getName() + "]";
	    }
	    
	    virtual ~ControlManifold(void)
	    {
	    }
	    
	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    T* as(void)
	    {
		/** \brief Make sure the type we are casting to is indeed a control manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlManifold*>));
		
		return static_cast<T*>(this);
	    }

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    const T* as(void) const
	    {	
		/** \brief Make sure the type we are casting to is indeed a control manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlManifold*>));
		
		return static_cast<const T*>(this);
	    }
	    	    
	    /** \brief Get the name of the manifold */
	    const std::string& getName(void) const
	    {
		return name_;
	    }
	    
	    /** \brief Set the name of the manifold */
	    void setName(const std::string &name)
	    {
		name_ = name;
	    }

	    /** \brief Return the state manifold this control manifold depends on */
	    const base::StateManifoldPtr& getStateManifold(void) const
	    {
		return stateManifold_;
	    }
	    
	    /** \brief Get the dimension of this manifold */
	    virtual unsigned int getDimension(void) const = 0;
	    
	    /** \brief Allocate memory for a control */
	    virtual Control* allocControl(void) const = 0;
	    
	    /** \brief Free the memory of a control */
	    virtual void freeControl(Control *control) const = 0;

	    /** \brief Copy a control to another */
	    virtual void copyControl(Control *destination, const Control *source) const = 0;
	    	    
	    /** \brief Check if two controls are the same */
	    virtual bool equalControls(const Control *control1, const Control *control2) const = 0;
	    
	    /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
	    virtual void nullControl(Control *control) const = 0;
	    
	    /** \brief Allocate a control sampler */
	    virtual ControlSamplerPtr allocControlSampler(void) const = 0;

	    /** \brief Propagate from a state, given a control, for some specified amount of time (the amount of time can
		also be negative, if canPropagateBackward() returns true)
		
		\note The pointer to the starting state and the result state may be the same.
	    */
	    virtual void propagate(const base::State *state, const Control* control, const double duration, base::State *result) const;
	    
	    /** \brief Some systems can only propagate forward in time (i.e., the duration argument for the propagate()
		function is always positive). If this is the case, this function should return false. Planners that need
		backward propagation (negative durations) will call this function to check. If backward propagation is 
		possible, this function should return true (this is the default). */
	    virtual bool canPropagateBackward(void) const;

	    /** \brief Set the function that performs state propagation */
	    void setPropagationFunction(const StatePropagationFn &fn);
	    
	    /** \brief Print a control to a stream */
	    virtual void printControl(const Control *control, std::ostream &out) const;

	    /** \brief Print the settings for this manifold to a stream */
	    virtual void printSettings(std::ostream &out) const;

	    /** \brief Perform final setup steps. This function is automatically called by the SpaceInformation */
	    virtual void setup(void);

	protected:
	    
	    /** \brief The state manifold controls can be applied to */
	    base::StateManifoldPtr stateManifold_;

	    /** \brief Function that can perform state propagation */
	    StatePropagationFn     statePropagation_;
	    
	    /** \brief The name of this manifold */
	    std::string            name_;
	};
	
	/** \brief A manifold to allow the composition of control manifolds */
	class CompoundControlManifold : public ControlManifold
	{
	public:

	    /** \brief Define the type of control allocated by this manifold */
	    typedef CompoundControl ControlType;
	    
	    /** \brief Constructor. The corresponding state manifold needs to be specified. */
	    CompoundControlManifold(const base::StateManifoldPtr &stateManifold) : ControlManifold(stateManifold), componentCount_(0), locked_(false)
	    {
	    }
	    
	    virtual ~CompoundControlManifold(void)
	    {
	    }

	    /** \brief Cast a component of this instance to a desired type. */
	    template<class T>
	    T* as(const unsigned int index) const
	    {
		/** \brief Make sure the type we are casting to is indeed a control manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlManifold*>));
		
		return static_cast<T*>(getSubManifold(index).get());
	    }
	    
	    /** \brief Adds a control manifold as a component of the compound manifold. */
	    virtual void addSubManifold(const ControlManifoldPtr &component);
	    
	    /** \brief Get the number of manifolds that make up the compound manifold */
	    unsigned int getSubManifoldCount(void) const;
	    
	    /** \brief Get a specific manifold from the compound manifold */
	    const ControlManifoldPtr& getSubManifold(const unsigned int index) const;

	    /** \brief Get a specific manifold from the compound manifold */
	    const ControlManifoldPtr& getSubManifold(const std::string &name) const;

	    virtual unsigned int getDimension(void) const;
	    
	    virtual Control* allocControl(void) const;
	    
	    virtual void freeControl(Control *control) const;

	    virtual void copyControl(Control *destination, const Control *source) const;
	    	    
	    virtual bool equalControls(const Control *control1, const Control *control2) const;
	    
	    virtual void nullControl(Control *control) const;
	    
	    virtual ControlSamplerPtr allocControlSampler(void) const;

	    virtual void propagate(const base::State *state, const Control* control, const double duration, base::State *result) const;

	    virtual bool canPropagateBackward(void) const;

	    virtual void printControl(const Control *control, std::ostream &out = std::cout) const;

	    virtual void printSettings(std::ostream &out) const;

	    virtual void setup(void);

	protected:

	    /** \brief Lock this manifold. This means no further
	     manifolds can be added as components.  This function can
	     be for instance called from the constructor of a manifold
	     that inherits from CompoundStateManifold to prevent the
	     user to add further components. */
	    void lock(void);
	    
	    /** \brief The component manifolds that make up the compound control space */
	    std::vector<ControlManifoldPtr> components_;

	    /** \brief The number of contained components */
	    unsigned int                    componentCount_;

	    /** \brief Flag indicating whether adding further componets is allowed or not */
	    bool                            locked_;
	};
    }
}
	    
#endif
