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

#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>
#include <cassert>

void ompl::geometric::KPIECE1::setup(void)
{
    Planner::setup();
    checkProjectionEvaluator(this, projectionEvaluator_);
    checkMotionLength(this, maxDistance_);
    
    tree_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::KPIECE1::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    tree_.grid.clear();
    tree_.size = 0;
    tree_.iteration = 1;
}

void ompl::geometric::KPIECE1::freeMemory(void)
{
    freeGridMotions(tree_.grid);
}

void ompl::geometric::KPIECE1::freeGridMotions(Grid &grid)
{
    for (Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
	freeCellData(it->second->data);
}

void ompl::geometric::KPIECE1::freeCellData(CellData *cdata)
{
    for (unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
	freeMotion(cdata->motions[i]);
    delete cdata;
}

void ompl::geometric::KPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
	si_->freeState(motion->state);
    delete motion;
}

bool ompl::geometric::KPIECE1::solve(const base::PlannerTerminationCondition &ptc)
{
    pis_.checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalRegion           *goal_r = dynamic_cast<base::GoalRegion*>(goal);
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    
    if (!goal)
    {
	msg_.error("Goal undefined");
	return false;
    }
    
    while (const base::State *st = pis_.nextStart())
    {
	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, st);
	addMotion(motion, 1.0);
    }
    
    if (tree_.grid.size() == 0)
    {
	msg_.error("There are no valid initial states!");
	return false;	
    }    

    if (!sampler_)
	sampler_ = si_->allocManifoldStateSampler();

    msg_.inform("Starting with %u states", tree_.size);
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();
    Grid::Coord  xcoord;    

    double improveValue = maxDistance_;

    while (ptc() == false)
    {
	tree_.iteration++;
	
	/* Decide on a state to expand from */
	Motion     *existing = NULL;
	Grid::Cell *ecell = NULL;
	selectMotion(existing, ecell);
	assert(existing);
	
	/* sample random state (with goal biasing) */
	if (rng_.uniform01() < goalBias_)
	{
	    if (goal_s && goal_s->canSample())
		goal_s->sampleGoal(xstate);
	    else
	    {
		if (approxsol && goal_r)
		{
		    si_->copyState(xstate, approxsol->state);
		    msg_.debug("Start Running HCIK (%f)...", improveValue);			
		    if (hcik_.tryToImprove(*goal_r, xstate, improveValue))
			improveValue /= 2.0;
		    else
			sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);
		    msg_.debug("End Running HCIK");			
		}
		else
		    sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);
	    }
	}
	else
	    sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);
	
	std::pair<base::State*, double> fail(xstate, 0.0);
	bool keep = si_->checkMotion(existing->state, xstate, fail);
	if (!keep && fail.second > minValidPathFraction_)
	    keep = true;
	
	if (keep)
	{
	    /* create a motion */
	    Motion *motion = new Motion(si_);
	    si_->copyState(motion->state, xstate);
	    motion->parent = existing;
	    
	    double dist = 0.0;
	    bool solved = goal->isSatisfied(motion->state, &dist);
	    addMotion(motion, dist);
	    
	    if (solved)
	    {
		approxdif = dist;
		solution = motion;
		break;
	    }
	    if (dist < approxdif)
	    {
		approxdif = dist;
		approxsol = motion;
	    }
	    ecell->data->score *= goodScoreFactor_;
	}
	else
	    ecell->data->score *= badScoreFactor_;

	tree_.grid.update(ecell);
    }
    
    bool approximate = false;
    if (solution == NULL)
    {	
	solution = approxsol;
	approximate = true;
    }
    
    if (solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion*> mpath;
	while (solution != NULL)
	{
	    mpath.push_back(solution);
	    solution = solution->parent;
	}

	/* set the solution path */
	PathGeometric *path = new PathGeometric(si_);
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	    path->states.push_back(si_->cloneState(mpath[i]->state));
	goal->setDifference(approxdif);
	goal->setSolutionPath(base::PathPtr(path), approximate);

	if (approximate)
	    msg_.warn("Found approximate solution");
    }

    si_->freeState(xstate);
    
    msg_.inform("Created %u states in %u cells (%u internal + %u external)", tree_.size, tree_.grid.size(),
		 tree_.grid.countInternal(), tree_.grid.countExternal());
    
    return goal->isAchieved();
}

bool ompl::geometric::KPIECE1::selectMotion(Motion* &smotion, Grid::Cell* &scell)
{
    scell = rng_.uniform01() < std::max(selectBorderFraction_, tree_.grid.fracExternal()) ?
	tree_.grid.topExternal() : tree_.grid.topInternal();

    // We are running on finite precision, so our update scheme will end up 
    // with 0 values for the score. This is where we fix the problem
    if (scell->data->score < std::numeric_limits<double>::epsilon())
    {
	std::vector<CellData*> content;
	content.reserve(tree_.grid.size());
	tree_.grid.getContent(content);
	for (std::vector<CellData*>::iterator it = content.begin() ; it != content.end() ; ++it)
	    (*it)->score += 1.0 + log((double)((*it)->iteration));
	tree_.grid.updateAll();
    }
    
    if (scell && !scell->data->motions.empty())
    {
	scell->data->selections++;
	smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
	return true;
    }
    else
	return false;
}

unsigned int ompl::geometric::KPIECE1::addMotion(Motion *motion, double dist)
{
    Grid::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell* cell = tree_.grid.getCell(coord);
    unsigned int created = 0;
    if (cell)
    {
	cell->data->motions.push_back(motion);
	cell->data->coverage += 1.0;
	tree_.grid.update(cell);
    }
    else
    {
	cell = tree_.grid.createCell(coord);
	cell->data = new CellData();
	cell->data->motions.push_back(motion);
	cell->data->coverage = 1.0;
	cell->data->iteration = tree_.iteration;
	cell->data->selections = 1;
	cell->data->score = (1.0 + log((double)(tree_.iteration))) / (1e-3 + dist);
	tree_.grid.add(cell);
	created = 1;
    }
    tree_.size++;
    return created;
}

void ompl::geometric::KPIECE1::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    
    std::vector<CellData*> cdata;
    tree_.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
	for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
	    data.recordEdge(cdata[i]->motions[j]->parent ? cdata[i]->motions[j]->parent->state : NULL, cdata[i]->motions[j]->state);
}
