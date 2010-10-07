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

#ifndef OMPL_BASE_STATE_MANIFOLD_
#define OMPL_BASE_STATE_MANIFOLD_

#include "ompl/base/State.h"
#include "ompl/base/ManifoldStateSampler.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {

	/** \brief Forward declaration of ompl::base::StateManifold */
	ClassForward(StateManifold);

	/** \class ompl::base::StateManifoldPtr 
	    \brief A boost shared pointer wrapper for ompl::base::StateManifold */


	/** \brief Representation of a space in which planning can be
	    performed. Topology specific sampling, interpolation and distance
	    are defined. 
	
	    See \ref implementStateManifold. */
	class StateManifold : private boost::noncopyable
	{
	public:

	    /** \brief Define the type of state allocated by this manifold */
	    typedef State StateType;
	    
	    /** \brief Constructor. Assigns a unique name to the manifold */
	    StateManifold(void);
	    
	    virtual ~StateManifold(void)
	    {
	    }
	    
	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    T* as(void)
	    {
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
		return static_cast<T*>(this);
	    }

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    const T* as(void) const
	    {	
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
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
	    
	    /** @name Functionality specific to the manifold (to be implemented)
		@{ */

	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const = 0;

	    /** \brief Get the maximum value a call to distance() can return */
	    virtual double getMaximumExtent(void) const = 0;
	    
	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(State *state) const = 0;
	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const State *state) const = 0;

	    /** \brief Copy a state to another. The memory of source and destination should not overlap. */
	    virtual void copyState(State *destination, const State *source) const = 0;
	    
	    /** \brief Computes distance to between two states. This value will always be between 0 and getMaximumExtent() */
	    virtual double distance(const State *state1, const State *state2) const = 0;
	    
	    /** \brief When performing discrete validation of motions,
		the length of the longest segment that does not
		require state validation needs to be speficied. This
		function returns this length, for this manifold, as a
		fraction of the manifold's maximum extent. */
	    virtual double getLongestValidSegmentFraction(void) const;

	    /** \brief When performing discrete validation of motions,
		the length of the longest segment that does not
		require state validation needs to be speficied. This
		function sets this length as a fraction of the manifold's
		maximum extent. */
	    virtual void setLongestValidSegmentFraction(double segmentFraction);
	    
	    /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2 */
	    virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const State *state1, const State *state2) const = 0;

	    /** \brief Computes the state that lies at time @e t in [0, 1] on the segment that connects @e from state to @e to state.
		The memory location of @e state is not required to be different from the memory of either
		@e from or @e to. */
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const = 0;
	    	    
	    /** \brief Allocate an instance of a uniform state sampler for this space */
	    virtual ManifoldStateSamplerPtr allocStateSampler(void) const = 0;

	    /** \brief Allocate a state that can store a point in the described space */
	    virtual State* allocState(void) const = 0;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(State *state) const = 0;
	    
	    /** @} */
	    
	    
	    /** @name Management of projections from this manifold to Euclidean spaces 
		@{ */
	    
	    /** \brief Register a projection for this manifold under a specified name */
	    void registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection);

	    /** \brief Register the default projection for this manifold */
	    void registerDefaultProjection(const ProjectionEvaluatorPtr &projection);
	    
	    /** \brief Get the projection registered under a specific name */
	    ProjectionEvaluatorPtr getProjection(const std::string &name) const;
	    
	    /** \brief Get the default projection */
	    ProjectionEvaluatorPtr getDefaultProjection(void) const;

	    /** \brief Check if a projection with a specified name is available */
	    bool hasProjection(const std::string &name) const;

	    /** \brief Check if a default projection is available */
	    bool hasDefaultProjection(void) const;

	    /** @} */
	
	    /** \brief Print a state to a stream */
	    virtual void printState(const State *state, std::ostream &out) const;
	    
	    /** \brief Print the settings for this manifold to a stream */
	    virtual void printSettings(std::ostream &out) const;

	    /** \brief Print the list of registered projections. This function is also called by printSettings() */
	    virtual void printProjections(std::ostream &out) const;
	    
	    /** \brief Perform final setup steps. This function is
		automatically called by the SpaceInformation. If any
		default projections are to be registered, this call
		will set them. It is safe to call this function
		multiple times. */
	    virtual void setup(void);
	    
	protected:

	    /** \brief The name used for the default projection */
	    static const std::string DEFAULT_PROJECTION_NAME;
	    
	    /** \brief Manifold name */
	    std::string                                   name_;
	    
	    /** \brief The extent of this manifold at the time setup() was called */
	    double                                        maxExtent_;
	    
	    /** \brief The fraction of the longest valid segment */
	    double                                        longestValidSegmentFraction_;
	    
	    /** \brief The longest valid segment at the time setup() was called */
	    double                                        longestValidSegment_;

	    /** \brief Interface used for console output */
	    msg::Interface                                msg_;
	    
	    /** \brief List of available projections */
	    std::map<std::string, ProjectionEvaluatorPtr> projections_;
	    
	};
	
	/** \brief A manifold to allow the composition of state manifolds */
    	class CompoundStateManifold : public StateManifold
	{
	public:
	    
	    /** \brief Define the type of state allocated by this manifold */
	    typedef CompoundState StateType;
	    
	    CompoundStateManifold(void) : StateManifold(), componentCount_(0), locked_(false)
	    {
	    }
	    
	    virtual ~CompoundStateManifold(void)
	    {
	    }

	    /** \brief Cast a component of this instance to a desired type. */
	    template<class T>
	    T* as(const unsigned int index) const
	    {
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
		return static_cast<T*>(getSubManifold(index).get());
	    }

	    /** \brief Cast a component of this instance to a desired type. */
	    template<class T>
	    T* as(const std::string &name) const
	    {
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
		return static_cast<T*>(getSubManifold(name).get());
	    }
	    
	    /** @name Management of contained manifolds 
		@{ */
	    
	    /** \brief Adds a new manifold as part of the compound space. For computing distances within the compound
		space, the weight of the component also needs to be specified. */
	    virtual void addSubManifold(const StateManifoldPtr &component, double weight);

	    /** \brief Replaces a specific manifold as part of the compound space. For computing distances within the compound
		space, the weight of the component also needs to be specified. */
	    virtual void replaceSubManifold(const unsigned int index, const StateManifoldPtr &component, double weight);

	    /** \brief Replaces a specific manifold as part of the compound space. For computing distances within the compound
		space, the weight of the component also needs to be specified. */
	    virtual void replaceSubManifold(const std::string &name, const StateManifoldPtr &component, double weight);
	    
	    /** \brief Get the number of manifolds that make up the compound manifold */
	    unsigned int getSubManifoldCount(void) const;
	    
	    /** \brief Get a specific manifold from the compound manifold */
	    const StateManifoldPtr& getSubManifold(const unsigned int index) const;

	    /** \brief Get a specific manifold from the compound manifold */
	    const StateManifoldPtr& getSubManifold(const std::string& name) const;

	    /** \brief Get the index of a specific manifold from the compound manifold */
	    unsigned int getSubManifoldIndex(const std::string& name) const;
	    
	    /** \brief Check if a specific submanifold is contained in this manifold */
	    bool hasSubManifold(const std::string &name) const;
	    
	    /** \brief Get a specific manifold's weight from the compound manifold (used in distance computation) */
	    double getSubManifoldWeight(const unsigned int index) const;

	    /** \brief Get a specific manifold's weight from the compound manifold (used in distance computation) */
	    double getSubManifoldWeight(const std::string &name) const;

	    /** \brief Set a specific manifold's weight in the compound manifold (used in distance computation) */
	    void setSubManifoldWeight(const unsigned int index, double weight);

	    /** \brief Set a specific manifold's weight in the compound manifold (used in distance computation) */
	    void setSubManifoldWeight(const std::string &name, double weight);
	    
	    /** @} */
	      
	    /** @name Functionality specific to the compound manifold
		@{ */

	    virtual unsigned int getDimension(void) const;

	    virtual double getMaximumExtent(void) const;

	    virtual void enforceBounds(State *state) const;
	    
	    virtual bool satisfiesBounds(const State *state) const;

	    virtual void copyState(State *destination, const State *source) const;

	    virtual double distance(const State *state1, const State *state2) const;

	    /** \brief When performing discrete validation of motions,
		the length of the longest segment that does not
		require state validation needs to be speficied. This
		function sets this length as a fraction of the manifold's
		maximum extent. The call is passed to all contained manifolds */
	    virtual void setLongestValidSegmentFraction(double segmentFraction);
	    
	    /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2.
		This is the max() of the counts returned by contained manifolds. */
	    virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;
	    
	    virtual bool equalStates(const State *state1, const State *state2) const;
	    
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const;
	    
	    virtual ManifoldStateSamplerPtr allocStateSampler(void) const;
	    
	    virtual State* allocState(void) const;
	    
	    virtual void freeState(State *state) const;	 
	    /** @} */

	    virtual void printState(const State *state, std::ostream &out) const;

	    virtual void printSettings(std::ostream &out) const;

	    virtual void setup(void);

	protected:

	    /** \brief Lock this manifold. This means no further
	     manifolds can be added as components.  This function can
	     be for instance called from the constructor of a manifold
	     that inherits from CompoundStateManifold to prevent the
	     user to add further components. */
	    void lock(void);
	    
	    /** \brief Allocate the state components. Called by allocState() */
	    void allocStateComponents(CompoundState *state) const;

	    /** \brief The component manifolds that make up the compound manifold */
	    std::vector<StateManifoldPtr> components_;
	    
	    /** \brief The number of components */
	    unsigned int                  componentCount_;

	    /** \brief The weight assigned to each component of the manifold when computing the compound distance */
	    std::vector<double>           weights_;
	    
	    /** \brief Flag indicating whether adding further componets is allowed or not */
	    bool                          locked_;
	    
	};
    }
}

#endif
