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

#include "ompl/kinematic/planners/est/EST.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>

bool ompl::kinematic::EST::solve(double solveTime)
{
    SpaceInformationKinematic      *si = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::Goal                   *goal = m_pdef->getGoal();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    unsigned int                   dim = si->getStateDimension();
    
    if (!goal)
    {
	m_msg.error("Goal undefined");
	return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);

    for (unsigned int i = m_addedStartStates ; i < m_pdef->getStartStateCount() ; ++i, ++m_addedStartStates)
    {
	const base::State *st = m_pdef->getStartState(i);
	if (si->satisfiesBounds(st) && si->isValid(st))
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, st);
	    addMotion(motion);
	}
	else
	    m_msg.error("Initial state is invalid!");
    }
    
    if (m_tree.grid.size() == 0)
    {
	m_msg.error("There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("Starting with %u states", m_tree.size);
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = new base::State(dim);
    
    while (time::now() < endTime)
    {
	/* Decide on a state to expand from */
	Motion *existing = selectMotion();
	assert(existing);
	
	/* sample random state (with goal biasing) */
	if (goal_s && m_rng.uniform01() < m_goalBias)
	    goal_s->sampleGoal(xstate);
	else
	    m_sCore().sampleNear(xstate, existing->state, range);
	
	if (si->checkMotion(existing->state, xstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, xstate);
	    motion->parent = existing;
	    
	    addMotion(motion);
	    double dist = 0.0;
	    bool solved = goal->isSatisfied(motion->state, &dist);
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
	PathKinematic *path = new PathKinematic(m_si);
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{   
	    base::State *st = new base::State(dim);
	    si->copyState(st, mpath[i]->state);
	    path->states.push_back(st);
	}
	goal->setDifference(approxdif);
	goal->setSolutionPath(path, approximate);

	if (approximate)
	    m_msg.warn("Found approximate solution");
    }

    delete xstate;
    
    m_msg.inform("Created %u states in %u cells", m_tree.size, m_tree.grid.size());
    
    return goal->isAchieved();
}

ompl::kinematic::EST::Motion* ompl::kinematic::EST::selectMotion(void)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    double prob = m_rng.uniform01() * (m_tree.grid.size() - 1);
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

void ompl::kinematic::EST::addMotion(Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
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

void ompl::kinematic::EST::getStates(std::vector</*const*/ base::State*> &states) const
{
    std::vector<MotionSet> motions;
    m_tree.grid.getContent(motions);
    states.resize(0);
    states.reserve(m_tree.size);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    states.push_back(motions[i][j]->state);    
}
