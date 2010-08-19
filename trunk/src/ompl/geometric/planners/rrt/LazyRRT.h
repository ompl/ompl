
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LAZY_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LAZY_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {
	
	/**
	   @anchor gLazyRRT
	   
	   @par Short description
	   
	   The basic idea of RRT is that it samples a random state @b qr
	   in the state space, then finds the state @b qc among the
	   previously seen states that is closest to @b qr and expands
	   from @b qc towards @b qr, until a state @b qm is reached and @b
	   qm is the new state to be visited. The difference between RRT
	   and LazyRRT is that when moving towards the new state @b qm,
	   LazyRRT does not check to make sure the path is valid. Instead,
	   it is optimistic and attempts to find a path as soon as
	   possible. Once a path is found, it is checked for collision. If
	   collisions are found, the invalid path segments are removed and
	   the search process is continued.       
	   
	   @par External documentation
	   @htmlonly
	   <a href="http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree">http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree</a>
	   <br>
	   <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html</a>
	   @endhtmlonly
	*/

	/** \brief Lazy RRT */
	class LazyRRT : public base::Planner
	{
	public:
	    
	    LazyRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "LazyRRT")
	    {
		type_ = base::PLAN_TO_GOAL_ANY;
		
		goalBias_ = 0.05;
		maxDistance_ = 0.0;
	    }
	    
	    virtual ~LazyRRT(void)
	    {
		freeMemory();
	    }

	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void);
	    
	    /** \brief Set the goal biasing.

		In the process of randomly selecting states in the state
		space to attempt to go towards, the algorithm may in fact
		choose the actual goal state, if it knows it, with some
		probability. This probability is a real number between 0.0
		and 1.0; its value should usually be around 0.05 and
		should not be too large. It is probably a good idea to use
		the default value. */
	    void setGoalBias(double goalBias)
	    {
		goalBias_ = goalBias;
	    }
	    
	    /** \brief Get the goal bias the planner is using */
	    double getGoalBias(void) const
	    {
		return goalBias_;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of the
		algorithm. It represents the maximum length of a
		motion to be added in the tree of motions. */
	    void setRange(double distance)
	    {
		maxDistance_ = distance;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return maxDistance_;
	    }

	    /** \brief Set a different nearest neighbors datastructure */
	    template<template<typename T> class NN>
	    void setNearestNeighbors(void)
	    {
		nn_.reset(new NN<Motion*>());
	    }
	    
	    virtual void setup(void);

	protected:

	    /** \brief Representation of a motion */
	    class Motion
	    {
	    public:
		
		Motion(void) : state(NULL), parent(NULL), valid(false)
		{
		}
		
		Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL), valid(false)
		{
		}
		
		~Motion(void)
		{
		}

		base::State          *state;
		Motion               *parent;
		bool                  valid;
		std::vector<Motion*>  children;
	    };

	    void freeMemory(void);

	    void removeMotion(Motion *motion);	
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return si_->distance(a->state, b->state);
	    }
	    
	    base::UniformStateSamplerPtr                   sampler_;
	    boost::shared_ptr< NearestNeighbors<Motion*> > nn_;	    
	    
	    double                                         goalBias_;
	    double                                         maxDistance_;	
	    RNG                                            rng_;
	
	};
	
    }
}

#endif
    
