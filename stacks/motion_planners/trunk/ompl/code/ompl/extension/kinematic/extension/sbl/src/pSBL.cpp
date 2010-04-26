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

#include "ompl/extension/kinematic/extension/sbl/pSBL.h"
#include <boost/thread.hpp>
#include <ros/console.h>

void ompl::kinematic::pSBL::threadSolve(unsigned int tid, unsigned int seed, ros::WallTime &endTime, SolutionInfo *sol)
{   
    RNG rng(seed);
    
    SpaceInformationKinematic *si   = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::GoalState           *goal = dynamic_cast<base::GoalState*>(si->getGoal());
    unsigned int               dim  = si->getStateDimension();
    
    std::vector<Motion*> solution;
    base::State *xstate = new base::State(dim);
    bool startTree = seed % 2;
    
    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    while (!sol->found && ros::WallTime::now() < endTime)
    {
	bool retry = true;
	while (retry && !sol->found && ros::WallTime::now() < endTime)
	{
	    m_removeList.lock.lock();
	    if (!m_removeList.motions.empty())
	    {
		if (m_loopLock.try_lock())
		{
		    retry = false;
		    std::map<Motion*, bool> seen;
		    for (unsigned int i = 0 ; i < m_removeList.motions.size() ; ++i)
			if (seen.find(m_removeList.motions[i].motion) == seen.end())
			    removeMotion(*m_removeList.motions[i].tree, m_removeList.motions[i].motion, seen);
		    m_removeList.motions.clear();
		    m_loopLock.unlock();
		}
	    }
	    else
		retry = false;
	    m_removeList.lock.unlock();
	}
	
	if (sol->found || ros::WallTime::now() > endTime)
	    break;
	
	m_loopLockCounter.lock();
	if (m_loopCounter == 0)
	    m_loopLock.lock();
	m_loopCounter++;
	m_loopLockCounter.unlock();
	

	TreeData &tree      = startTree ? m_tStart : m_tGoal;
	startTree = !startTree;
	TreeData &otherTree = startTree ? m_tStart : m_tGoal;
	
	Motion *existing = selectMotion(rng, tree);
	m_sCoreArray.sCore[tid]->sampleNear(xstate, existing->state, range);
	
	/* create a motion */
	Motion *motion = new Motion(dim);
	si->copyState(motion->state, xstate);
	motion->parent = existing;

	existing->lock.lock();
	existing->children.push_back(motion);
	existing->lock.unlock();
	
	addMotion(tree, motion);

	if (checkSolution(rng, !startTree, tree, otherTree, motion, solution))
	{
	    sol->lock.lock();
	    if (!sol->found)
	    {
		sol->found = true;
		PathKinematic *path = new PathKinematic(m_si);
		for (unsigned int i = 0 ; i < solution.size() ; ++i)
		{
		    base::State *st = new base::State(dim);
		    si->copyState(st, solution[i]->state);
		    path->states.push_back(st);
		}
		goal->setDifference(0.0);
		goal->setSolutionPath(path);
	    }
	    sol->lock.unlock();
	}

	
	m_loopLockCounter.lock();
	m_loopCounter--;
	if (m_loopCounter == 0)
	    m_loopLock.unlock();
	m_loopLockCounter.unlock();
    }
    
    delete xstate;
    
}

bool ompl::kinematic::pSBL::solve(double solveTime)
{
    SpaceInformationKinematic *si   = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::GoalState           *goal = dynamic_cast<base::GoalState*>(si->getGoal());
    unsigned int               dim  = si->getStateDimension();
    
    if (!goal)
    {
	ROS_ERROR("pSBL: Unknown type of goal (or goal undefined)");
	return false;
    }
    
    ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(solveTime);
    
    if (m_tStart.size == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, si->getStartState(i));
	    if (si->satisfiesBounds(motion->state) && si->isValid(motion->state))
	    {
		motion->valid = true;
		addMotion(m_tStart, motion);
	    }
	    else
	    {
		ROS_ERROR("pSBL: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_tGoal.size == 0)
    {	   
	Motion *motion = new Motion(dim);
	si->copyState(motion->state, goal->state);
	if (si->satisfiesBounds(motion->state) && si->isValid(motion->state))
	{
	    motion->valid = true;
	    addMotion(m_tGoal, motion);
	}
	else
	{
	    ROS_ERROR("pSBL: Goal state is invalid!");
	    delete motion;
	}
    }
    
    if (m_tStart.size == 0 || m_tGoal.size == 0)
    {
	ROS_ERROR("pSBL: Motion planning trees could not be initialized!");
	return false;
    }
    
    ROS_INFO("pSBL: Starting with %d states", (int)(m_tStart.size + m_tGoal.size));
    
    SolutionInfo sol;
    sol.found = false;
    m_loopCounter = 0;
    
    std::vector<boost::thread*> th(m_threadCount);
    for (unsigned int i = 0 ; i < m_threadCount ; ++i)
	th[i] = new boost::thread(boost::bind(&pSBL::threadSolve, this, i, m_rng.uniformInt(1, 10000000), endTime, &sol));
    for (unsigned int i = 0 ; i < m_threadCount ; ++i)
    {
	th[i]->join();
	delete th[i];
    }
        
    ROS_INFO("pSBL: Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", m_tStart.size + m_tGoal.size, m_tStart.size, m_tGoal.size,
	     m_tStart.grid.size() + m_tGoal.grid.size(), m_tStart.grid.size(), m_tGoal.grid.size());
    
    return goal->isAchieved();
}

bool ompl::kinematic::pSBL::checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);

    otherTree.lock.lock();    
    Grid<MotionSet>::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data.empty())
    {
	Motion *connectOther          = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
	otherTree.lock.unlock();    
	
	SpaceInformationKinematic *si = static_cast<SpaceInformationKinematic*>(m_si);
	Motion *connect               = new Motion(si->getStateDimension());

	si->copyState(connect->state, connectOther->state);
	connect->parent = motion;

	motion->lock.lock();
	motion->children.push_back(connect);
	motion->lock.unlock();

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
    else
	otherTree.lock.unlock();    
    
    return false;
}

bool ompl::kinematic::pSBL::isPathValid(TreeData &tree, Motion *motion)
{
    std::vector<Motion*>       mpath;
    SpaceInformationKinematic *si = static_cast<SpaceInformationKinematic*>(m_si);
    
    /* construct the solution path */
    while (motion != NULL)
    {  
	mpath.push_back(motion);
	motion = motion->parent;
    }
    
    bool result = true;
    
    /* check the path */
    for (int i = mpath.size() - 1 ; result && i >= 0 ; --i)
    {
	mpath[i]->lock.lock();
	if (!mpath[i]->valid)
	{
	    if (si->checkMotionSubdivision(mpath[i]->parent->state, mpath[i]->state))
		mpath[i]->valid = true;
	    else
	    {
		// remember we need to remove this motion
		PendingRemoveMotion prm;
		prm.tree = &tree;
		prm.motion = mpath[i];
		m_removeList.lock.lock();
		m_removeList.motions.push_back(prm);
		m_removeList.lock.unlock();
		result = false;
	    }
	}
	mpath[i]->lock.unlock();
    }
    
    return result;
}

ompl::kinematic::pSBL::Motion* ompl::kinematic::pSBL::selectMotion(RNG &rng, TreeData &tree)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    tree.lock.lock();
    double prob = rng.uniform() * (tree.grid.size() - 1);
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
    ompl::kinematic::pSBL::Motion* result = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
    tree.lock.unlock();
    return result;
}

void ompl::kinematic::pSBL::removeMotion(TreeData &tree, Motion *motion, std::map<Motion*, bool> &seen)
{
    /* remove from grid */
    seen[motion] = true;

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
	removeMotion(tree, motion->children[i], seen);
    }
    
    delete motion;
}

void ompl::kinematic::pSBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
    tree.lock.lock();
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
    tree.lock.unlock();
}

void ompl::kinematic::pSBL::getStates(std::vector<const base::State*> &states) const
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

void ompl::kinematic::pSBL::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);		
    m_threadCount = nthreads;
    m_sCoreArray.setCount(m_threadCount);
}
