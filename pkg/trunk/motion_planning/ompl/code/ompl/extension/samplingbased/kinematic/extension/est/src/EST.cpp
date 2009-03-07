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

#include "ompl/extension/samplingbased/kinematic/extension/est/EST.h"

bool ompl::sb::EST::solve(double solveTime)
{
    SpaceInformationKinematic *si = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    GoalRegion            *goal_r = dynamic_cast<GoalRegion*>(si->getGoal());
    GoalState             *goal_s = dynamic_cast<GoalState*>(si->getGoal());
    unsigned int              dim = si->getStateDimension();
    
    if (!goal_s && !goal_r)
    {
	m_msg.error("EST: Unknown type of goal (or goal undefined)");
	return false;
    }
    
    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);

    if (m_tree.grid.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, dynamic_cast<State*>(si->getStartState(i)));
	    if (si->isValid(motion->state))
		addMotion(motion);
	    else
	    {
		m_msg.error("EST: Initial state is in collision!");
		delete motion;
	    }	
	}
    }
    
    if (m_tree.grid.size() == 0)
    {
	m_msg.error("EST: There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("EST: Starting with %u states", m_tree.size);
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = INFINITY;
    State  *xstate    = new State(dim);
    
    while (time_utils::Time::now() < endTime)
    {
	/* Decide on a state to expand from */
	Motion *existing = selectMotion();
	assert(existing);
	
	/* sample random state (with goal biasing) */
	if (goal_s && m_rng.uniform(0.0, 1.0) < m_goalBias)
	    si->copyState(xstate, static_cast<State*>(goal_s->state));
	else
	    m_sCore.sampleNear(xstate, existing->state, range);
	
	if (si->checkMotionSubdivision(existing->state, xstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, xstate);
	    motion->parent = existing;

	    addMotion(motion);
	    double dist = 0.0;
	    bool solved = goal_r->isSatisfied(motion->state, &dist);
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
	}
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
	PathKinematic *path = new PathKinematic(static_cast<SpaceInformation*>(m_si));
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{   
	    State *st = new State(dim);
	    si->copyState(st, mpath[i]->state);
	    path->states.push_back(st);
	}
	goal_r->setDifference(approxdif);
	goal_r->setSolutionPath(path, approximate);

	if (approximate)
	    m_msg.warn("EST: Found approximate solution");
    }

    delete xstate;
    
    m_msg.inform("EST: Created %u states in %u cells", m_tree.size, m_tree.grid.size());
    
    return goal_r->isAchieved();
}

ompl::sb::EST::Motion* ompl::sb::EST::selectMotion(void)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    double prob = m_rng.uniform() * (m_tree.grid.size() - 1);
    for (Grid<MotionSet>::iterator it = m_tree.grid.begin(); it != m_tree.grid.end() ; ++it)
    {
	sum += (double)(m_tree.size - it->second->data.size()) / (double)m_tree.size;
	if (prob < sum)
	{
	    cell = it->second;
	    break;
	}
    }
    if (!cell && m_tree.grid.size() > 0)
	cell = m_tree.grid.begin()->second;
    return cell && !cell->data.empty() ? cell->data[m_rng.uniformInt(0, cell->data.size() - 1)] : NULL;
}

void ompl::sb::EST::computeCoordinates(const Motion* motion, Grid<MotionSet>::Coord &coord)
{
    coord.resize(m_projectionDimension);
    double projection[m_projectionDimension];
    (*m_projectionEvaluator)(static_cast<base::State*>(motion->state), projection);
    
    for (unsigned int i = 0 ; i < m_projectionDimension; ++i)
	coord[i] = (int)trunc(projection[i]/m_cellDimensions[i]);
}

void ompl::sb::EST::addMotion(Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    computeCoordinates(motion, coord);
    Grid<MotionSet>::Cell* cell = m_tree.grid.getCell(coord);
    if (cell)
	cell->data.push_back(motion);
    else
    {
	cell = m_tree.grid.createCell(coord);
	cell->data.push_back(motion);
	m_tree.grid.add(cell);
    }
    m_tree.size++;
}
