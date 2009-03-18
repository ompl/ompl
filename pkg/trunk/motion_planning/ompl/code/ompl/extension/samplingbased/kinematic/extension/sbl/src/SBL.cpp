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

#include "ompl/extension/samplingbased/kinematic/extension/sbl/SBL.h"

bool ompl::sb::SBL::solve(double solveTime)
{
    SpaceInformationKinematic *si   = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    GoalState                 *goal = dynamic_cast<GoalState*>(si->getGoal());
    unsigned int               dim  = si->getStateDimension();
    
    if (!goal)
    {
	m_msg.error("SBL: Unknown type of goal (or goal undefined)");
	return false;
    }
    
    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);
    
    if (m_tStart.size == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, dynamic_cast<State*>(si->getStartState(i)));
	    if (si->isValid(motion->state))
	    {
		motion->valid = true;
		addMotion(m_tStart, motion);
	    }
	    else
	    {
		m_msg.error("SBL: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_tGoal.size == 0)
    {	   
	Motion *motion = new Motion(dim);
	si->copyState(motion->state, static_cast<State*>(goal->state));
	if (si->isValid(motion->state))
	{
	    motion->valid = true;
	    addMotion(m_tGoal, motion);
	}
	else
	{
	    m_msg.error("SBL: Goal state is invalid!");
	    delete motion;
	}
    }
    
    if (m_tStart.size == 0 || m_tGoal.size == 0)
    {
	m_msg.error("SBL: Motion planning trees could not be initialized!");
	return false;
    }
    
    m_msg.inform("SBL: Starting with %d states", (int)(m_tStart.size + m_tGoal.size));
    
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
	
	Motion *existing = selectMotion(tree);
	assert(existing);
	m_sCore.sampleNear(xstate, existing->state, range);
	
	/* create a motion */
	Motion *motion = new Motion(dim);
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
    
    m_msg.inform("SBL: Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", m_tStart.size + m_tGoal.size, m_tStart.size, m_tGoal.size,
		 m_tStart.grid.size() + m_tGoal.grid.size(), m_tStart.grid.size(), m_tGoal.grid.size());
    
    return goal->isAchieved();
}

bool ompl::sb::SBL::checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(static_cast<base::State*>(motion->state), coord);
    Grid<MotionSet>::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data.empty())
    {
	SpaceInformationKinematic *si = static_cast<SpaceInformationKinematic*>(m_si);
	Motion *connectOther          = cell->data[m_rng.uniformInt(0, cell->data.size() - 1)];
	Motion *connect               = new Motion(si->getStateDimension());
	
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

bool ompl::sb::SBL::isPathValid(TreeData &tree, Motion *motion)
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

ompl::sb::SBL::Motion* ompl::sb::SBL::selectMotion(TreeData &tree)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    double prob = m_rng.uniform() * (tree.grid.size() - 1);
    for (Grid<MotionSet>::iterator it = tree.grid.begin(); it != tree.grid.end() ; ++it)
    {
	sum += (double)(tree.size - it->second->data.size()) / (double)tree.size;
	if (prob < sum)
	{
	    cell = it->second;
	    break;
	}
    }
    if (!cell && tree.grid.size() > 0)
	cell = tree.grid.begin()->second;
    return cell && !cell->data.empty() ? cell->data[m_rng.uniformInt(0, cell->data.size() - 1)] : NULL;
}

void ompl::sb::SBL::removeMotion(TreeData &tree, Motion *motion)
{
    /* remove from grid */
    
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(static_cast<base::State*>(motion->state), coord);
    Grid<MotionSet>::Cell* cell = tree.grid.getCell(coord);
    if (cell)
    {
	for (unsigned int i = 0 ; i < cell->data.size(); ++i)
	    if (cell->data[i] == motion)
	    {
		cell->data.erase(cell->data.begin() + i);
		tree.size--;
		break;
	    }
	if (cell->data.empty())
	{
	    tree.grid.remove(cell);
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

void ompl::sb::SBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(static_cast<base::State*>(motion->state), coord);
    Grid<MotionSet>::Cell* cell = tree.grid.getCell(coord);
    if (cell)
	cell->data.push_back(motion);
    else
    {
	cell = tree.grid.createCell(coord);
	cell->data.push_back(motion);
	tree.grid.add(cell);
    }
    tree.size++;
}
