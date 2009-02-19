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

#include "ompl/extension/samplingbased/kinematic/extension/kpiece/KPIECE1.h"

bool ompl::KPIECE1::solve(double solveTime)
{
    SpaceInformationKinematic_t                          si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(si->getGoal());
    SpaceInformationKinematic::GoalStateKinematic_t  goal_s = dynamic_cast<SpaceInformationKinematic::GoalStateKinematic_t>(si->getGoal());
    unsigned int                                        dim = si->getStateDimension();
    
    if (!goal_s && !goal_r)
    {
	m_msg.error("KPIECE1: Unknown type of goal (or goal undefined)");
	return false;
    }
    
    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);

    if (m_tree.grid.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, dynamic_cast<SpaceInformationKinematic::StateKinematic_t>(si->getStartState(i)));
	    if (si->isValid(motion->state))
		addMotion(motion, 1, 1.0);
	    else
	    {
		m_msg.error("KPIECE1: Initial state is in collision!");
		delete motion;
	    }	
	}
    }
    
    if (m_tree.grid.size() == 0)
    {
	m_msg.error("KPIECE1: There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("KPIECE1: Starting with %u states", m_tree.size);
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    Motion                                     *solution  = NULL;
    Motion                                     *approxsol = NULL;
    double                                      approxdif = INFINITY;
    SpaceInformationKinematic::StateKinematic_t xstate    = new SpaceInformationKinematic::StateKinematic(dim);

    double improveValue = 0.01;
    unsigned int iteration = 1;

    while (time_utils::Time::now() < endTime)
    {
	iteration++;
	
	/* Decide on a state to expand from */
	Motion     *existing = NULL;
	Grid::Cell *ecell = NULL;
	selectMotion(existing, ecell);
	assert(existing);
	
	/* sample random state (with goal biasing) */
	if (m_rng.uniform(0.0, 1.0) < m_goalBias)
	{
	    if (goal_s)
		si->copyState(xstate, goal_s->state);
	    else
	    {
		if (approxsol)
		{
		    si->copyState(xstate, approxsol->state);
		    if (!m_hcik.tryToImprove(xstate, improveValue))
		    {
			si->sampleNear(xstate, existing->state, range);
			improveValue /= 2.0;
		    }
		}
		else
		    si->sampleNear(xstate, existing->state, range);
	    }
	}
	else
	    si->sampleNear(xstate, existing->state, range);
	
	double failTime = 0.0;
	bool keep = si->checkMotionIncremental(existing->state, xstate, xstate, &failTime);
	if (!keep && failTime > m_minValidPathPercentage)
	    keep = true;
	
	if (keep)
	{
	    /* create a motion */
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, xstate);
	    motion->parent = existing;

	    double dist = 0.0;
	    bool solved = goal_r->isSatisfied(motion->state, &dist);
	    addMotion(motion, iteration, dist);
	    
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
	    ecell->data->score *= m_goodScoreFactor;
	}
	else
	    ecell->data->score *= m_badScoreFactor;
	
	m_tree.grid.update(ecell);
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
	SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{   
	    SpaceInformationKinematic::StateKinematic_t st = new SpaceInformationKinematic::StateKinematic(dim);
	    si->copyState(st, mpath[i]->state);
	    path->states.push_back(st);
	}
	goal_r->setDifference(approxdif);
	goal_r->setSolutionPath(path, approximate);

	if (approximate)
	    m_msg.warn("KPIECE1: Found approximate solution");
    }

    delete xstate;
    
    m_msg.inform("KPIECE1: Created %u states in %u cells (%u internal + %u external)", m_tree.size, m_tree.grid.size(),
		 m_tree.grid.countInternal(), m_tree.grid.countExternal());
    
    return goal_r->isAchieved();
}

bool ompl::KPIECE1::selectMotion(Motion* &smotion, Grid::Cell* &scell)
{
    scell = m_rng.uniform() < std::max(m_selectBorderPercentage, m_tree.grid.fracExternal()) ?
	m_tree.grid.topExternal() : m_tree.grid.topInternal();
    if (scell && !scell->data->motions.empty())
    {
	scell->data->selections++;
	smotion = scell->data->motions[m_rng.halfNormalInt(0, scell->data->motions.size() - 1)];
	return true;
    }
    else
	return false;
}

void ompl::KPIECE1::computeCoordinates(const Motion *motion, Grid::Coord &coord)
{
    coord.resize(m_projectionDimension);
    double projection[m_projectionDimension];
    (*m_projectionEvaluator)(static_cast<SpaceInformation::State*>(motion->state), projection);
    
    for (unsigned int i = 0 ; i < m_projectionDimension; ++i)
	coord[i] = (int)trunc(projection[i]/m_cellDimensions[i]);
}

unsigned int ompl::KPIECE1::addMotion(Motion *motion, unsigned int iteration, double dist)
{
    Grid::Coord coord;
    computeCoordinates(motion, coord);
    Grid::Cell* cell = m_tree.grid.getCell(coord);
    unsigned int created = 0;
    if (cell)
    {
	cell->data->motions.push_back(motion);
	cell->data->coverage += 1.0;
	m_tree.grid.update(cell);
    }
    else
    {
	cell = m_tree.grid.createCell(coord);
	cell->data = new CellData();
	cell->data->motions.push_back(motion);
	cell->data->coverage = 1.0;
	cell->data->iteration = iteration;
	cell->data->selections = 1;
	cell->data->score = 1.0 / (1e-3 + dist);
	m_tree.grid.add(cell);
	created = 1;
    }
    m_tree.size++;
    return created;
}
