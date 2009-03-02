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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_RRT_LAZY_RRT_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_RRT_LAZY_RRT_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include <vector>

/** Main namespace */
namespace ompl
{
    
    /** Forward class declaration */
    ForwardClassDeclaration(LazyRRT);
    
    /**
       @subsubsection LazyRRT Lazy Rapidly-exploring Random Trees (LazyRRT)
       
       @par Short description
       
       The basic idea of RRT is that it samples a random state @b qr
       in the state space, then finds the state @b qc among the
       previously seen states that is closest to @b qr and expands
       from @b qc towards @b qr, until a state @b qm is reached and @b
       qm is the new state to be visited. The difference between RRT
       and LazyRRT is that when moving towards the new state @qm,
       LazyRRT does not check to make sure the path is valid. Instead,
       it is optimistic and attempts to find a path as soon as
       possible. Once a path is found, it is checked for collision. If
       collisions are found, the invalid path segments are removed and
       the search process is continued.       
       
       @par External documentation
       @link http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
       @link http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html
    */
    class LazyRRT : public Planner
    {
    public:

        LazyRRT(SpaceInformation_t si) : Planner(si),
	                                 m_sCore(dynamic_cast<SpaceInformationKinematic*>(si))
	{
	    m_type = PLAN_TO_GOAL_STATE | PLAN_TO_GOAL_REGION;
	    m_dEval = new DistanceFunction(dynamic_cast<SpaceInformationKinematic_t>(si));
	    m_nn.setDistanceFunction(m_dEval);
	    m_goalBias = 0.05;
	    m_rho = 0.5;
	}

	virtual ~LazyRRT(void)
	{
	    freeMemory();
	    if (m_dEval)
		delete m_dEval;
	}
	
	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
	    m_nn.clear();
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
	
	/** Get the goal bias the planner is using */
	double getGoalBias(void) const
	{
	    return m_goalBias;
	}
		
	/** Set the range the planner is supposed to use. This
	    parameter greatly influences the runtime of the
	    algorithm. It is probably a good idea to find what a good
	    value is for each model the planner is used for. The basic
	    idea of RRT is that it samples a random state qr in the
	    state space, then finds the state qc among the previously
	    seen states that is closest to qr and expands from qc
	    towards qr, until a state qm is reached and qm is the new
	    state to be visited. The range parameter influences how
	    this qm along the path between qc and qr is chosen. qr may
	    be too far, and it may not be best to have qm = qr all the
	    time (range = 1.0 implies qm=qr. range should be less than
	    1.0). However, in a large space, it is also good to leave
	    the neighborhood of qc (range = 0.0 implies qm=qc and no
	    progress is made. rande should be larger than
	    0.0). Multiple values of this range parameter should be
	    tried until a suitable one is found. */
	void setRange(double rho)
	{
	    m_rho = rho;
	}

	/** Get the range the planner is using */
	double getRange(void) const
	{
	    return m_rho;
	}
	
    protected:
	ForwardClassDeclaration(Motion);
	
	class Motion
	{
	public:
	    
	    Motion(void)
	    {
		parent = NULL;
		state  = NULL;
		valid  = false;
	    }
	    
	    Motion(unsigned int dimension)
	    {
		state  = new SpaceInformationKinematic::StateKinematic(dimension);
		parent = NULL;
		valid  = false;
	    }
	    
	    ~Motion(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    SpaceInformationKinematic::StateKinematic_t state;
	    Motion_t                                    parent;
	    std::vector<Motion_t>                       children;
	    bool                                        valid;
	};

	void freeMemory(void)
	{
	    std::vector<Motion_t> motions;
	    m_nn.list(motions);
	    for (unsigned int i = 0 ; i < motions.size() ; ++i)
		delete motions[i];
	}

	void removeMotion(Motion_t motion);	
	
	class DistanceFunction : public NearestNeighborsSqrtApprox<Motion_t>::DistanceFunction
	{
	public:
	    DistanceFunction(SpaceInformationKinematic_t si)
	    {
		assert(si);
		m_si = si;
	    }
	    
	    double operator()(const Motion_t &a, const Motion_t &b) const
	    {
		return m_si->distance(a->state, b->state);
	    }
	protected:
	    
	    SpaceInformationKinematic_t m_si;
	    
	};

	SpaceInformationKinematic::SamplingCore m_sCore;
	
	NearestNeighborsSqrtApprox<Motion_t>    m_nn;
	DistanceFunction                       *m_dEval;
	
	double                                  m_goalBias;
	double                                  m_rho;	
	random_utils::RNG                       m_rng;
	
    };

}

#endif
