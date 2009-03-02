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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_EST_EST_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_EST_EST_

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include <vector>

namespace ompl
{
    
    /** Forward class declaration */
    ForwardClassDeclaration(EST);
    
    /**
       @subsubsection EST Expansive Space Trees (EST)
       
       @par Short description
       
       EST attempts to detect the less explored area of the space
       through the use of a grid imposed on a projection of the state
       space. Using this information, EST continues tree expansion
       primarily from less explored areas.
       
       @par External documentation
       Path planning in expansive configuration spaces
       Hsu, D.; Latombe, J.-C.; Motwani, R.
       IEEE International Conference on Robotics and Automation, 1997.
       Volume 3, Issue , 20-25 Apr 1997 Page(s):2719 - 2726 vol.3
    */
    class EST : public Planner
    {
    public:

        EST(SpaceInformation_t si) : Planner(si)
	{
	    m_type = PLAN_TO_GOAL_STATE | PLAN_TO_GOAL_REGION;
	    m_projectionEvaluator = NULL;
	    m_projectionDimension = 0;
	    m_goalBias = 0.05;
	    m_rho = 0.5;
	}

	virtual ~EST(void)
	{
	    freeMemory();
	}
	
	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
	    m_tree.grid.clear();
	    m_tree.size = 0;
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
	    value is for each model the planner is used for. The range
	    parameter influences how this @b qm along the path between
	    @b qc and @b qr is chosen. @b qr may be too far, and it
	    may not be best to have @b qm = @b qr all the time (range
	    = 1.0 implies @b qm = @b qr. range should be less than
	    1.0). However, in a large space, it is also good to leave
	    the neighborhood of @b qc (range = 0.0 implies @b qm = @b
	    qc and no progress is made. rande should be larger than
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

	/** Set the projection evaluator. This class is able to
	    compute the projection of a given state. The simplest
	    option is to use an orthogonal projection; see
	    OrthogonalProjectionEvaluator */
	void setProjectionEvaluator(ProjectionEvaluator_t projectionEvaluator)
	{
	    m_projectionEvaluator = projectionEvaluator;
	}

	ProjectionEvaluator_t getProjectionEvaluator(void) const
	{
	    return m_projectionEvaluator;
	}

	virtual void setup(void)
	{
	    assert(m_projectionEvaluator);
	    m_projectionDimension = m_projectionEvaluator->getDimension();
	    assert(m_projectionDimension > 0);
	    m_projectionEvaluator->getCellDimensions(m_cellDimensions);
	    assert(m_cellDimensions.size() == m_projectionDimension);
	    m_tree.grid.setDimension(m_projectionDimension);
	    Planner::setup();
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
	    }
	    
	    Motion(unsigned int dimension)
	    {
		state  = new SpaceInformationKinematic::StateKinematic(dimension);
		parent = NULL;
	    }
	    
	    virtual ~Motion(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    SpaceInformationKinematic::StateKinematic_t state;
	    Motion_t                                    parent;
	    
	};

	typedef std::vector<Motion_t> MotionSet;

	struct TreeData
	{
	    TreeData(void) : grid(0)
	    {
		size = 0;
	    }
	    
	    Grid<MotionSet> grid;
	    unsigned int    size;
	};
	
	void freeMemory(void)
	{
	    for (Grid<MotionSet>::iterator it = m_tree.grid.begin(); it != m_tree.grid.end() ; ++it)
	    {
		for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
		    delete it->second->data[i];
	    }
	}

	void addMotion(Motion_t motion);
	Motion_t selectMotion(void);
	void computeCoordinates(const Motion_t motion, Grid<MotionSet>::Coord &coord);
	
	TreeData               m_tree;
	
	ProjectionEvaluator   *m_projectionEvaluator;
	unsigned int           m_projectionDimension;
	std::vector<double>    m_cellDimensions;

	double                 m_goalBias;
	double                 m_rho;	
	mutable random_utils::RNG   m_rng;	
    };

}

#endif
