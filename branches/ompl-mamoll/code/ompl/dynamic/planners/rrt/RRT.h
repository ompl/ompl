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

#ifndef OMPL_DYNAMIC_PLANNERS_RRT_RRT_
#define OMPL_DYNAMIC_PLANNERS_RRT_RRT_

#include "ompl/base/Planner.h"
#include "ompl/dynamic/SpaceInformationControlsIntegrator.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"

namespace ompl
{
    
    namespace dynamic
    {
	
	/**
	   @anchor dRRT
	   
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
	    
	    RRT(SpaceInformationControlsIntegrator *si) : base::Planner(si),
							  m_sCore(si),
							  m_cCore(si)
	    {
		m_type = base::PLAN_TO_GOAL_ANY;
		m_msg.setPrefix("RRT");
		
		m_nn.setDistanceFunction(boost::bind(&RRT::distanceFunction, this, _1, _2));
		m_goalBias = 0.05;
		m_hintBias = 0.75;
		m_addedStartStates = 0;
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
	    virtual void clear(void)
	    {
		freeMemory();
		m_nn.clear();
		m_addedStartStates = 0;
	    }
	    
	    /** In the process of randomly selecting states in the state
		space to attempt to go towards, the algorithm may in fact
		choose the actual goal state, if it knows it, with some
		probability. This probability is a real number between 0.0
		and 1.0; its value should usually be around 0.05 and
		should not be too large. It is probably a good idea to use
		the default value. */
	    void setGoalBias(double goalBias)
	    {
		m_goalBias = goalBias;
	    }
	    
	    /** \brief Get the goal bias the planner is using */
	    double getGoalBias(void) const
	    {
		return m_goalBias;
	    }

	    /** \brief If a kinematic path is given as hint, this
		function sets the percentage of how often this path is
		used as hint */
	    void setHintBias(double hintBias)
	    {
		m_hintBias = hintBias;
	    }
	    
	    /** \brief Get the hint bias */
	    double getHintBias(void) const
	    {
		return m_hintBias;
	    }	    
	    
	    virtual void getStates(std::vector</*const*/ base::State*> &states) const;
	    
	protected:
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
		{
		}
		
		Motion(unsigned int sdim, unsigned int cdim) : state(new base::State(sdim)), control(new Control(cdim)), steps(0), parent(NULL)
		{
		}
		
		~Motion(void)
		{
		    if (state)
			delete state;
		    if (control)
			delete control;
		}
		
		base::State       *state;
		Control           *control;
		unsigned int       steps;
		Motion            *parent;
		
	    };
	    
	    void freeMemory(void)
	    {
		std::vector<Motion*> motions;
		m_nn.list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		    delete motions[i];
	    }
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return m_si->distance(a->state, b->state);
	    }
	    
	    base::StateSamplerInstance          m_sCore;
	    ControlSamplerInstance              m_cCore;
 
	    NearestNeighborsSqrtApprox<Motion*> m_nn;
	    unsigned int                        m_addedStartStates;
	    
	    double                              m_goalBias;
	    double                              m_hintBias;
	    RNG                                 m_rng;	
	};
	
    }
}

#endif
    
