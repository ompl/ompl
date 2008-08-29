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

/** \Author Ioan Sucan */

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_SBL_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_SBL_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include <vector>

namespace ompl
{

    ForwardClassDeclaration(SBL);
    
    class SBL : public Planner
    {
    public:

	/** Forward class declaration */
	ForwardClassDeclaration(ProjectionEvaluator);	
	
	
	/** Abstract definition for a class computing projections */
	class ProjectionEvaluator
	{
	public:
	    /** Destructor */
	    virtual ~ProjectionEvaluator(void)
	    {
	    }
	    
	    /** Return the dimension of the projection defined by this evaluator */
	    virtual unsigned int getDimension(void) const = 0;
	    
	    /** Compute the projection as an array of double values */
	    virtual void operator()(const SpaceInformationKinematic::StateKinematic_t state, double *projection) = 0;
	};

	/** Definition for a class computing orthogonal projections */
        class OrthogonalProjectionEvaluator : public ProjectionEvaluator
	{
	public:
	    
	    OrthogonalProjectionEvaluator(const std::vector<unsigned int> &components) : ProjectionEvaluator()
	    {
		m_components = components;
	    }

	    virtual unsigned int getDimension(void) const
	    {
		return m_components.size();
	    }
	    
	    virtual void operator()(const SpaceInformationKinematic::StateKinematic_t state, double *projection)
	    {
		for (unsigned int i = 0 ; i < m_components.size() ; ++i)
		    projection[i] = state->values[m_components[i]];
	    }
	    
	protected:
	    
	    std::vector<unsigned int> m_components;
	    
	};	
	
	
        SBL(SpaceInformation_t si) : Planner(si)
	{
	    random_utils::random_init(&m_rngState);
	    m_projectionEvaluator = NULL;
	    m_projectionDimension = 0;
	    m_rho = 0.1;
	}

	virtual ~SBL(void)
	{
	    freeMemory();
	}
	
	void setProjectionEvaluator(ProjectionEvaluator_t projectionEvaluator)
	{
	    m_projectionEvaluator = projectionEvaluator;
	}
	
	void setCellDimensions(std::vector<double> &cellDimensions)
	{
	    m_cellDimensions = cellDimensions;
	}
	
	virtual void setup(void)
	{
	    assert(m_projectionEvaluator);
	    m_projectionDimension = m_projectionEvaluator->getDimension();
	    assert(m_projectionDimension > 0);
	    assert(m_cellDimensions.size() == m_projectionDimension);
	    Planner::setup();
	}

	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
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
	    if (m_projectionEvaluator)
		delete m_projectionEvaluator;
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
	random_utils::rngState m_rngState;	
    };

}

#endif
