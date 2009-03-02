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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_KPIECE_KPIECE1_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_KPIECE_KPIECE1_

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridX.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include "ompl/extension/samplingbased/kinematic/AllocatorStateKinematic.h"
#include "ompl/extension/samplingbased/kinematic/extension/ik/HCIK.h"
#include <vector>

namespace ompl
{
    
    /** Forward class declaration */
    ForwardClassDeclaration(KPIECE1);
    
       /**
       @subsubsection KPIECE1 KPIECE (Kinematic Planning by
       Interior-Exterior Cell Exploration) with one level of
       discretization
       
       @par Short description
       
       KPIECE is a tree-based planner that uses a discretization
       (multiple levels, in general) to guide the exploration of the
       continous space. 
       
       @par External documentation

       Ioan A. Sucan, Lydia E. Kavraki, Kinodynamic Planning by
       Interior-Exterior Cell Exploration, International Workshop on
       the Algorithmic Foundations of Robotics, 2008.
       http://ioan.sucan.info

    */

    class KPIECE1 : public Planner
    {
    public:

        KPIECE1(SpaceInformation_t si) : Planner(si),
	                                 m_hcik(dynamic_cast<SpaceInformationKinematic_t>(si))
	{
	    m_type = PLAN_TO_GOAL_STATE | PLAN_TO_GOAL_REGION;
	    m_projectionEvaluator = NULL;
	    m_projectionDimension = 0;
	    m_goalBias = 0.05;
	    m_selectBorderPercentage = 0.9;
	    m_badScoreFactor = 0.5;
	    m_goodScoreFactor = 0.9;
	    m_minValidPathPercentage = 0.2;
	    m_rho = 0.5;
	    m_tree.grid.onCellUpdate(computeImportance, NULL);
	    m_hcik.setMaxImproveSteps(50);
	}

	virtual ~KPIECE1(void)
	{
	    freeMemory();
	}
	
	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
	    m_tree.grid.clear();
	    m_tree.size = 0;
	    m_tree.iteration = 1;
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
	    Motion                                     *parent;
	    
	};

	struct CellData
	{
	    CellData(void)
	    {
		coverage = 0.0;
		selections = 1;
		score = 1.0;
		iteration = 0;
		importance = 0.0;
	    };
	    
	    ~CellData(void)
	    {
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		    delete motions[i];
	    }

	    std::vector<Motion*> motions;
	    double               coverage;
	    unsigned int         selections;
	    double               score;
	    unsigned int         iteration;
	    double               importance;
	};
	
	struct OrderCellsByImportance
	{
	    bool operator()(const CellData * const a, const CellData * const b) const
	    {
		return a->importance > b->importance;
	    }
	};
	
	typedef GridX<CellData*, OrderCellsByImportance> Grid;
	
	struct TreeData
	{
	    TreeData(void) : grid(0)
	    {
		size = 0;
		iteration = 1;
	    }
	    
	    Grid         grid;
	    unsigned int size;
	    unsigned int iteration;
	};

	static void computeImportance(Grid::Cell *cell, void*)
	{
	    CellData &cd = *(cell->data);
	    cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
	}
	
	void freeMemory(void)
	{
	    freeGridMotions(m_tree.grid);
	}

	void freeGridMotions(Grid &grid)
	{
	    for (Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
		delete it->second->data;
	}
	
	unsigned int addMotion(Motion* motion, double dist);
	bool selectMotion(Motion* &smotion, Grid::Cell* &scell);
	void computeCoordinates(const Motion* motion, Grid::Coord &coord);
	
	HCIK                   m_hcik;
	TreeData               m_tree;
	
	ProjectionEvaluator   *m_projectionEvaluator;
	unsigned int           m_projectionDimension;
	std::vector<double>    m_cellDimensions;

	double                 m_minValidPathPercentage;
	double                 m_goodScoreFactor;
	double                 m_badScoreFactor;
	double                 m_selectBorderPercentage;
	double                 m_goalBias;
	double                 m_rho;	
	random_utils::RNG      m_rng;	
    };

}

#endif
