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

#include "ompl/extension/samplingbased/kinematic/extension/kpiece/LBKPIECE1.h"

bool ompl::sb::LBKPIECE1::solve(double solveTime)
{
    SpaceInformationKinematic *si = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    GoalState               *goal = dynamic_cast<GoalState*>(si->getGoal());
    unsigned int              dim = si->getStateDimension();
    
    if (!goal)
    {
	m_msg.error("LBKPIECE1: Unknown type of goal (or goal undefined)");
	return false;
    }
    
    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);
    
    if (m_tStart.size == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion* motion = new Motion(dim);
	    si->copyState(motion->state, dynamic_cast<State*>(si->getStartState(i)));
	    if (si->isValid(motion->state))
	    {
		motion->valid = true;
		addMotion(m_tStart, motion);
	    }
	    else
	    {
		m_msg.error("LBKPIECE1: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_tGoal.size == 0)
    {	   
	Motion* motion = new Motion(dim);
	si->copyState(motion->state, static_cast<State*>(goal->state));
	if (si->isValid(motion->state))
	{
	    motion->valid = true;
	    addMotion(m_tGoal, motion);
	}
	else
	{
	    m_msg.error("LBKPIECE1: Goal state is invalid!");
	    delete motion;
	}
    }
    
    if (m_tStart.size == 0 || m_tGoal.size == 0)
    {
	m_msg.error("LBKPIECE1: Motion planning trees could not be initialized!");
	return false;
    }
    
    m_msg.inform("LBKPIECE1: Starting with %d states", (int)(m_tStart.size + m_tGoal.size));
    
    std::vector<Motion*> solution;
    State *xstate    = new State(dim);
    bool   startTree = true;
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    while (time_utils::Time::now() < endTime)
    {
	TreeData &tree      = startTree ? m_tStart : m_tGoal;
	startTree = !startTree;
	TreeData &otherTree = startTree ? m_tStart : m_tGoal;
	tree.iteration++;
	
	Motion* existing = selectMotion(tree);
	assert(existing);
	m_sCore.sampleNear(xstate, existing->state, range);
	
	/* create a motion */
	Motion* motion = new Motion(dim);
	si->copyState(motion->state, xstate);
	motion->parent = existing;
	existing->children.push_back(motion);
	
	addMotion(tree, motion);
	
	if (checkSolution(!startTree, tree, otherTree, motion, solution))
	{
	    PathKinematic *path = new PathKinematic(static_cast<SpaceInformation*>(m_si));
	    for (unsigned int i = 0 ; i < solution.size() ; ++i)
	    {
		State *st = new State(dim);
		si->copyState(st, solution[i]->state);
		path->states.push_back(st);
	    }
	    
	    goal->setDifference(0.0);
	    goal->setSolutionPath(path);
	    break;
	}
    }
    
    delete xstate;
    
    m_msg.inform("LBKPIECE1: Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", 
		 m_tStart.size + m_tGoal.size, m_tStart.size, m_tGoal.size,
		 m_tStart.grid.size() + m_tGoal.grid.size(), m_tStart.grid.size(), m_tGoal.grid.size());
    
    return goal->isAchieved();
}

bool ompl::sb::LBKPIECE1::checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion* motion, std::vector<Motion*> &solution)
{
    Grid::Coord coord;
    m_projectionEvaluator->computeCoordinates(static_cast<base::State*>(motion->state), coord);
    Grid::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data->motions.empty())
    {
	SpaceInformationKinematic *si = static_cast<SpaceInformationKinematic*>(m_si);
	Motion* connectOther          = cell->data->motions[m_rng.uniformInt(0, cell->data->motions.size() - 1)];
	Motion* connect               = new Motion(si->getStateDimension());
	
	si->copyState(connect->state, connectOther->state);
	connect->parent = motion;
	motion->children.push_back(connect);
	addMotion(tree, connect);
	
	if (isPathValid(tree, connect) && isPathValid(otherTree, connectOther))
	{
	    /* extract the motions and put them in solution vector */
	    
	    std::vector<Motion*> mpath1;
	    while (motion != NULL)
	    {
		mpath1.push_back(motion);
		motion = motion->parent;
	    }
	    
	    std::vector<Motion*> mpath2;
	    while (connectOther != NULL)
	    {
		mpath2.push_back(connectOther);
		connectOther = connectOther->parent;
	    }
	    
	    if (!start)
		mpath1.swap(mpath2);
	    
	    for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
		solution.push_back(mpath1[i]);
	    solution.insert(solution.end(), mpath2.begin(), mpath2.end());
	    
	    return true;
	}
    }
    return false;
}

bool ompl::sb::LBKPIECE1::isPathValid(TreeData &tree, Motion* motion)
{
    std::vector<Motion*>       mpath;
    SpaceInformationKinematic *si = static_cast<SpaceInformationKinematic*>(m_si);
    
    /* construct the solution path */
    while (motion != NULL)
    {  
	mpath.push_back(motion);
	motion = motion->parent;
    }
    
    /* check the path */
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	if (!mpath[i]->valid)
	{
	    if (si->checkMotionSubdivision(mpath[i]->parent->state, mpath[i]->state))
		mpath[i]->valid = true;
	    else
	    {
		removeMotion(tree, mpath[i]);
		return false;
	    }
	}
    return true;
}

ompl::sb::LBKPIECE1::Motion* ompl::sb::LBKPIECE1::selectMotion(TreeData &tree)
{
    Grid::Cell* cell = m_rng.uniform() < std::max(m_selectBorderPercentage, tree.grid.fracExternal()) ?
	tree.grid.topExternal() : tree.grid.topInternal();
    if (cell && !cell->data->motions.empty())
    {
	cell->data->selections++;
	tree.grid.update(cell);
	return cell->data->motions[m_rng.halfNormalInt(0, cell->data->motions.size() - 1)];
    }
    else
	return NULL;
}

void ompl::sb::LBKPIECE1::removeMotion(TreeData &tree, Motion* motion)
{
    /* remove from grid */
    
    Grid::Coord coord;
    m_projectionEvaluator->computeCoordinates(static_cast<base::State*>(motion->state), coord);
    Grid::Cell* cell = tree.grid.getCell(coord);
    if (cell)
    {
	for (unsigned int i = 0 ; i < cell->data->motions.size(); ++i)
	    if (cell->data->motions[i] == motion)
	    {
		cell->data->motions.erase(cell->data->motions.begin() + i);
		tree.size--;
		break;
	    }
	if (cell->data->motions.empty())
	{
	    tree.grid.remove(cell);
	    delete cell->data;
	    tree.grid.destroyCell(cell);
	}
    }
    
    /* remove self from parent list */
    
    if (motion->parent)
    {
	for (unsigned int i = 0 ; i < motion->parent->children.size() ; ++i)
	    if (motion->parent->children[i] == motion)
	    {
		motion->parent->children.erase(motion->parent->children.begin() + i);
		break;
	    }
    }    
    
    /* remove children */
    for (unsigned int i = 0 ; i < motion->children.size() ; ++i)
    {
	motion->children[i]->parent = NULL;
	removeMotion(tree, motion->children[i]);
    }
    
    delete motion;
}

void ompl::sb::LBKPIECE1::addMotion(TreeData &tree, Motion* motion)
{
    Grid::Coord coord;
    m_projectionEvaluator->computeCoordinates(static_cast<base::State*>(motion->state), coord);
    Grid::Cell* cell = tree.grid.getCell(coord);
    if (cell)
    {
	cell->data->motions.push_back(motion);
	cell->data->coverage += 1.0;
	tree.grid.update(cell);
    }
    else
    {
	cell = tree.grid.createCell(coord);
	cell->data = new CellData();
	cell->data->motions.push_back(motion);
	cell->data->coverage = 1.0;
	cell->data->iteration = tree.iteration;
	cell->data->selections = 1;
	tree.grid.add(cell);
    }
    tree.size++;
}
