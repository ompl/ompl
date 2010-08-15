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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include <vector>

namespace ompl
{
    
    namespace geometric
    {
	
	/**
	   @anchor gLBKPIECE1
	   with one level of discretization
	   
	   @par Short description
	   
	   KPIECE is a tree-based planner that uses a discretization
	   (multiple levels, in general) to guide the exploration of the
	   continous space. 
	   
	   @par External documentation
	   
	   Ioan A. Sucan, Lydia E. Kavraki, Kinodynamic Planning by
	   Interior-Exterior Cell Exploration, International Workshop on
	   the Algorithmic Foundations of Robotics, 2008.

	   @htmlonly
	   <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">http://ioan.sucan.ro/files/pubs/wafr2008.pdf</a>
	   @endhtmlonly
	*/
	
	/** \brief Lazy Bi-directional KPIECE */
	class LBKPIECE1 : public base::Planner
	{
	public:
	    
	    LBKPIECE1(const base::SpaceInformationPtr &si) : base::Planner(si, "LBKPIECE1")
	    {
		type_ = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;
		
		selectBorderPercentage_ = 0.9;
		maxDistance_ = 0.0;
		
		tStart_.grid.onCellUpdate(computeImportance, NULL);
		tGoal_.grid.onCellUpdate(computeImportance, NULL);
	    }
	    
	    virtual ~LBKPIECE1(void)
	    {
		freeMemory();
	    }
	    
	    /** \brief Set the projection evaluator. This class is
		able to compute the projection of a given state. */
	    void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
	    {
		projectionEvaluator_ = projectionEvaluator;
	    }

	    /** \brief Set the projection evaluator (select one from
		the ones registered with the state manifold). */
	    void setProjectionEvaluator(const std::string &name)
	    {
		projectionEvaluator_ = si_->getStateManifold()->getProjection(name);
	    }

	    /** \brief Get the projection evaluator. */	    
	    const base::ProjectionEvaluatorPtr& getProjectionEvaluator(void) const
	    {
		return projectionEvaluator_;
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
	    
	    /** \brief Set the percentage of time for focusing on the
		border. This is the minimum percentage used to select
		cells that are exterior (minimum because if 95% of cells
		are on the border, they will be selected with 95%
		chance, even if this percentage is set to 90%)*/
	    void setBorderPercentage(double bp)
	    {
		selectBorderPercentage_ = bp;
	    }
	    
	    double getBorderPercentage(void) const
	    {
		return selectBorderPercentage_;
	    }
	    
	    virtual void setup(void);
	    
	    virtual bool solve(double solveTime);
	    virtual void clear(void);

	    virtual void getPlannerData(base::PlannerData &data) const;

	protected:
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : root(NULL), state(NULL), parent(NULL), valid(false)
		{
		}
		
		Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL), valid(false)
		{
		}
		
		~Motion(void)
		{
		}
		
		const base::State   *root;
		base::State         *state;
		Motion              *parent;
		bool                 valid;
		std::vector<Motion*> children;
	    };
	    
	    struct CellData
	    {
		CellData(void) : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0)
		{
		}
		
		~CellData(void)
		{
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
	    
	    typedef GridB<CellData*, OrderCellsByImportance> Grid;
	    
	    struct TreeData
	    {
		TreeData(void) : grid(0), size(0), iteration(1)
		{
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
	    
	    void freeMemory(void);
	    void freeGridMotions(Grid &grid);
	    void freeCellData(CellData *cdata);
	    void freeMotion(Motion *motion);
	    
	    void addMotion(TreeData &tree, Motion* motion);
	    Motion* selectMotion(TreeData &tree);	
	    void removeMotion(TreeData &tree, Motion* motion);
	    bool isPathValid(TreeData &tree, Motion* motion);
	    bool checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion* motion, std::vector<Motion*> &solution);
	    
	    base::StateSamplerPtr                      sampler_;

	    base::ProjectionEvaluatorPtr               projectionEvaluator_;
	    
	    TreeData                                   tStart_;
	    TreeData                                   tGoal_;
	    
	    double                                     selectBorderPercentage_;
	    double                                     maxDistance_;
	    RNG                                        rng_;	
	};
	
    }
}


#endif
