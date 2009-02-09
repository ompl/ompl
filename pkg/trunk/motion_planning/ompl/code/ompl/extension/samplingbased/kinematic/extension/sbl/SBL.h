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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_SBL_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_SBL_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include "ompl/extension/samplingbased/kinematic/ProjectionEvaluator.h"
#include <vector>

/** Main namespace */
namespace ompl
{

    /** Forward class declaration */
    ForwardClassDeclaration(SBL);
    
    /**
       @subsubsection SBL Single-query Bi-directional Lazy collision checking planner (SBL)
       
       @par Short description
       
       SBL is a tree-based motion planner that attempts to grow two
       trees at once: one grows from the starting state and the other
       from the goal state. Attempts are made to connect these trees
       at every step of the expansion. If they are connected, a
       solution path is obtained. However, this solution path is not
       certain to be valid (the lazy part of the algorithm) so it is
       checked for validity. If invalid parts are found, they are
       removed from the tree and exploration of the state space
       continues until a solution is found. 

       To guide the exploration, and additional grid data structure is
       maintained. Grid cells contain states that have been previously
       visited. When deciding which state to use for further
       expansion, this grid is used and least filled grid cells have
       most chances of being selected. The grid is usually imposed on
       a projection of the state space. This projection needs to be
       set before using the planner.
       
       @par External documentation

       G. Sanchez and J.C. Latombe.A Single-Query Bi-Directional
       Probabilistic Roadmap Planner with Lazy Collision
       Checking. Int. Symposium on Robotics Research (ISRR'01), Lorne,
       Victoria, Australia, November 2001.
    */
    class SBL : public Planner
    {
    public:

        SBL(SpaceInformation_t si) : Planner(si)
	{
	    m_type = PLAN_TO_GOAL_STATE;
	    m_projectionEvaluator = NULL;
	    m_projectionDimension = 0;
	    m_rho = 0.5;
	}

	virtual ~SBL(void)
	{
	    freeMemory();
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

	/** Define the dimension (each component) of a grid cell. The
	    number of dimensions set here must be the same as the
	    dimension of the projection computed by the projection
	    evaluator. */
	void setCellDimensions(const std::vector<double> &cellDimensions)
	{
	    m_cellDimensions = cellDimensions;
	}

	void getCellDimensions(std::vector<double> &cellDimensions) const
	{
	    cellDimensions = m_cellDimensions;
	}
	
	/** Set the range the planner is supposed to use. This
	    parameter greatly influences the runtime of the
	    algorithm. It is probably a good idea to find what a good
	    value is for each model the planner is used for. The basic
	    idea of SBL is that it samples a random state around a
	    state that was already added to the tree. The distance
	    withing which this new state is sampled is controled by
	    the range. This should be a value larger than 0.0 and less
	    than 1.0 */
	void setRange(double rho)
	{
	    m_rho = rho;
	}
	
	/** Get the range the planner is using */
	double getRange(void) const
	{
	    return m_rho;
	}
	
	virtual void setup(void)
	{
	    assert(m_projectionEvaluator);
	    m_projectionDimension = m_projectionEvaluator->getDimension();
	    assert(m_projectionDimension > 0);
	    assert(m_cellDimensions.size() == m_projectionDimension);
	    m_tStart.grid.setDimension(m_projectionDimension);
	    m_tGoal.grid.setDimension(m_projectionDimension);
	    Planner::setup();
	}

	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
	    
	    m_tStart.grid.clear();
	    m_tStart.size = 0;
	    
	    m_tGoal.grid.clear();
	    m_tGoal.size = 0;	    
	}
	
    protected:

       	ForwardClassDeclaration(Motion);
	
	typedef std::vector<Motion_t> MotionSet;	
	
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
	    
	    virtual ~Motion(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    SpaceInformationKinematic::StateKinematic_t state;
	    Motion_t                                    parent;
	    bool                                        valid;
	    MotionSet                                   children;
	};
	
	struct TreeData
	{
	    TreeData(void)
	    {
		size = 0;
	    }
	    
	    Grid<MotionSet> grid;
	    unsigned int    size;
	};
	
	void freeMemory(void)
	{
	    freeGridMotions(m_tStart.grid);
	    freeGridMotions(m_tGoal.grid);
	}

	void freeGridMotions(Grid<MotionSet> &grid)
	{
	    for (Grid<MotionSet>::iterator it = grid.begin(); it != grid.end() ; ++it)
	    {
		for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
		    delete it->second->data[i];
	    }
	}
	
	void addMotion(TreeData &tree, Motion_t motion);
	Motion_t selectMotion(TreeData &tree);	
	void removeMotion(TreeData &tree, Motion_t motion);
	void computeCoordinates(const Motion_t motion, Grid<MotionSet>::Coord &coord);
	bool isPathValid(TreeData &tree, Motion_t motion);
	bool checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion_t motion, std::vector<Motion_t> &solution);

	ProjectionEvaluator   *m_projectionEvaluator;
	unsigned int           m_projectionDimension;
	std::vector<double>    m_cellDimensions;
		
	TreeData               m_tStart;
	TreeData               m_tGoal;
	
	double                 m_rho;	
	random_utils::RNGSet   m_rng;	
    };

}

#endif
