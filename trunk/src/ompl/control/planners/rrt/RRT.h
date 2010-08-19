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

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRT_
#define OMPL_CONTROL_PLANNERS_RRT_RRT_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    
    namespace control
    {
	
	/**
	   @anchor cRRT
	   
	   @par Short description
	   
	   The basic idea of RRT is that it samples a random state @b qr
	   in the state space, then finds the state @b qc among the
	   previously seen states that is closest to @b qr and expands
	   from @b qc towards @b qr, until a state @b qm is reached and @b
	   qm is the new state to be visited.
	   
	   
	   @par External documentation

	   @htmlonly
	   <a href="http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree">http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree</a>
	   <br>
	   <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html</a>
	   @endhtmlonly 
	*/

	/** \brief Rapidly-exploring Random Tree */
	class RRT : public base::Planner
	{
	public:
	    
	    RRT(const SpaceInformationPtr &si) : base::Planner(si, "RRT")
	    {
		type_ = base::PLAN_TO_GOAL_ANY;
		siC_ = si.get();

		goalBias_ = 0.05;
	    }
	    
	    virtual ~RRT(void)
	    {
		freeMemory();
	    }
	    
	    /** \brief Continue solving for some amount of time. Return true if solution was found. */
	    virtual bool solve(double solveTime);
	    
	    /** \brief Clear datastructures. Call this function if the
		input data to the planner has changed and you do not
		want to continue planning */
	    virtual void clear(void);
	    
	    /** In the process of randomly selecting states in the state
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

	    virtual void getPlannerData(base::PlannerData &data) const;

	    /** \brief Set a different nearest neighbors datastructure */
	    template<template<typename T> class NN>
	    void setNearestNeighbors(void)
	    {
		nn_.reset(new NN<Motion*>());
	    }
	    
	    virtual void setup(void);
	    
	protected:
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
		{
		}
		
		Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
		{
		}
		
		~Motion(void)
		{
		}
		
		base::State       *state;
		Control           *control;
		unsigned int       steps;
		Motion            *parent;		
	    };
	    
	    void freeMemory(void)
	    {
		std::vector<Motion*> motions;
		nn_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
		    if (motions[i]->state)
			si_->freeState(motions[i]->state);
		    if (motions[i]->control)
			siC_->freeControl(motions[i]->control);
		    delete motions[i];
		}
	    }
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return si_->distance(a->state, b->state);
	    }
	    
	    base::ManifoldStateSamplerPtr                  sampler_;
	    ControlSamplerPtr                              controlSampler_;
	    const SpaceInformation                        *siC_;
	    
	    boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

	    double                                         goalBias_;
	    RNG                                            rng_;
	};
	
    }
}

#endif
    
