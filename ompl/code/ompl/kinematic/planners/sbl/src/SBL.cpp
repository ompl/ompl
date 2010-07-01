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

#include "ompl/kinematic/planners/sbl/SBL.h"
#include "ompl/base/GoalSampleableRegion.h"

bool ompl::kinematic::SBL::solve(double solveTime)
{
    SpaceInformationKinematic *si    = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(m_pdef->getGoal());
    unsigned int               dim   = si->getStateDimension();
    
    if (!goal)
    {
	m_msg.error("Unknown type of goal (or goal undefined)");
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
	    motion->valid = true;
	    motion->root = st;
	    addMotion(m_tStart, motion);
	}
	else
	    m_msg.error("Initial state is invalid!");
    }
    
    if (m_tStart.size == 0)
    {
	m_msg.error("Motion planning start tree could not be initialized!");
	return false;
    }

    if (goal->maxSampleCount() <= 0)
    {
	m_msg.error("Insufficient states in sampleable goal region");
	return false;
    }
    
    m_msg.inform("Starting with %d states", (int)(m_tStart.size + m_tGoal.size));
    
    std::vector<Motion*> solution;
    base::State *xstate = new base::State(dim); 
    base::State *gstate = new base::State(dim);

    bool      startTree = true;
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    while (time::now() < endTime)
    {
	TreeData &tree      = startTree ? m_tStart : m_tGoal;
	startTree = !startTree;
	TreeData &otherTree = startTree ? m_tStart : m_tGoal;
	
	// if there are any goals left to sample
	if (m_sampledGoalsCount < goal->maxSampleCount())
	{
	    // if we have not sampled too many goals already
	    if (m_tGoal.size == 0 || m_sampledGoalsCount < m_tGoal.size / 2)
	    {
		bool firstAttempt = true;
		while ((m_tGoal.size == 0 || firstAttempt) && m_sampledGoalsCount < goal->maxSampleCount() && time::now() < endTime)
		{
		    firstAttempt = false;
		    goal->sampleGoal(gstate);
		    m_sampledGoalsCount++;
		    if (si->satisfiesBounds(gstate) && si->isValid(gstate))
		    {
			Motion* motion = new Motion(dim);
			si->copyState(motion->state, gstate);
			motion->root = motion->state;
			motion->valid = true;
			addMotion(m_tGoal, motion);
		    }
		}

		if (m_tGoal.size == 0)
		{
		    m_msg.error("Unable to sample any valid states for goal tree");
		    break;
		}
	    }
	}
	
	Motion *existing = selectMotion(tree);
	assert(existing);
	m_sCore().sampleNear(xstate, existing->state, range);
	
	/* create a motion */
	Motion *motion = new Motion(dim);
	si->copyState(motion->state, xstate);
	motion->parent = existing;
	motion->root = existing->root;
	existing->children.push_back(motion);
	
	addMotion(tree, motion);
	
	if (checkSolution(!startTree, tree, otherTree, motion, solution))
	{
	    PathKinematic *path = new PathKinematic(m_si);
	    for (unsigned int i = 0 ; i < solution.size() ; ++i)
	    {
		base::State *st = new base::State(dim);
		si->copyState(st, solution[i]->state);
		path->states.push_back(st);
	    }
	    
	    goal->setDifference(0.0);
	    goal->setSolutionPath(path);
	    break;
	}
    }
    
    delete gstate;
    delete xstate;
    
    m_msg.inform("Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", m_tStart.size + m_tGoal.size, m_tStart.size, m_tGoal.size,
		 m_tStart.grid.size() + m_tGoal.grid.size(), m_tStart.grid.size(), m_tGoal.grid.size());
    
    return goal->isAchieved();
}

bool ompl::kinematic::SBL::checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
    Grid<MotionSet>::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data.empty())
    {
	Motion *connectOther = cell->data[m_rng.uniformInt(0, cell->data.size() - 1)];
	
	if (m_pdef->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root, start ? connectOther->root : motion->root))
	{
	    SpaceInformationKinematic *si = static_cast<SpaceInformationKinematic*>(m_si);
	    Motion *connect               = new Motion(si->getStateDimension());
	    
	    si->copyState(connect->state, connectOther->state);
	    connect->parent = motion;
	    connect->root = motion->root;
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
    }
    return false;
}

bool ompl::kinematic::SBL::isPathValid(TreeData &tree, Motion *motion)
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
	    if (si->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		mpath[i]->valid = true;
	    else
	    {
		removeMotion(tree, mpath[i]);
		return false;
	    }
	}
    return true;
}

ompl::kinematic::SBL::Motion* ompl::kinematic::SBL::selectMotion(TreeData &tree)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    double prob = m_rng.uniform01() * (tree.grid.size() - 1);
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

void ompl::kinematic::SBL::removeMotion(TreeData &tree, Motion *motion)
{
    /* remove from grid */
    
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
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

void ompl::kinematic::SBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
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

void ompl::kinematic::SBL::clear(void)
{
    freeMemory();
    
    m_tStart.grid.clear();
    m_tStart.size = 0;
    
    m_tGoal.grid.clear();
    m_tGoal.size = 0;
    
    m_sampledGoalsCount = 0;
    m_addedStartStates = 0;
}

void ompl::kinematic::SBL::getStates(std::vector</*const*/ base::State*> &states) const
{
    states.resize(0);
    states.reserve(m_tStart.size + m_tGoal.size);
    
    std::vector<MotionSet> motions;
    m_tStart.grid.getContent(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    states.push_back(motions[i][j]->state);    

    motions.clear();
    m_tGoal.grid.getContent(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    states.push_back(motions[i][j]->state);    
}
