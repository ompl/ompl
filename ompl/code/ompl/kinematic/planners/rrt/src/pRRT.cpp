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

#include "ompl/kinematic/planners/rrt/pRRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <boost/thread/thread.hpp>
#include <ros/console.h>

void ompl::kinematic::pRRT::threadSolve(unsigned int tid, ros::WallTime &endTime, SolutionInfo *sol)
{
    SpaceInformationKinematic  *si     = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::Goal                 *goal   = si->getGoal();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(si->getGoal());
    unsigned int                   dim = si->getStateDimension();

    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);

    Motion *rmotion   = new Motion(dim);
    base::State *rstate = rmotion->state;
    base::State *xstate = new base::State(dim);

    while (sol->solution == NULL && ros::WallTime::now() < endTime)
    {
	/* sample random state (with goal biasing) */
	if (goal_s && m_sCoreArray[tid]->getRNG().uniform01() < m_goalBias)
	    goal_s->sampleGoal(rstate);
	else
	    m_sCoreArray[tid]->sample(rstate);
	
	/* find closest state in the tree */
	m_nnLock.lock();
	Motion *nmotion = m_nn.nearest(rmotion);
	m_nnLock.unlock();

	/* find state to add */
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    double diff = rmotion->state->values[i] - nmotion->state->values[i];
	    xstate->values[i] = fabs(diff) < range[i] ? rmotion->state->values[i] : nmotion->state->values[i] + diff * m_rho;
	}
	
	if (si->checkMotion(nmotion->state, xstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, xstate);
	    motion->parent = nmotion;

	    m_nnLock.lock();
	    m_nn.add(motion);
	    m_nnLock.unlock();
	    
	    double dist = 0.0;
	    bool solved = goal->isSatisfied(motion->state, &dist);
	    if (solved)
	    {
		sol->lock.lock();
		sol->approxdif = dist;
		sol->solution = motion;
		sol->lock.unlock();
		break;
	    }
	    if (dist < sol->approxdif)
	    {
		sol->lock.lock();
		if (dist < sol->approxdif)
		{
		    sol->approxdif = dist;
		    sol->approxsol = motion;
		}
		sol->lock.unlock();
	    }
	}
    }
    
    delete xstate;
    delete rmotion;
}

bool ompl::kinematic::pRRT::solve(double solveTime)
{
    SpaceInformationKinematic *si   = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::GoalRegion          *goal = dynamic_cast<base::GoalRegion*>(si->getGoal()); 
    unsigned int                dim = si->getStateDimension();
    
    if (!goal)
    {
	ROS_ERROR("pRRT: Goal undefined");
	return false;
    }
    
    ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(solveTime);

    if (m_nn.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, si->getStartState(i));
	    if (si->satisfiesBounds(motion->state) && si->isValid(motion->state))
		m_nn.add(motion);
	    else
	    {
		ROS_ERROR("pRRT: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_nn.size() == 0)
    {
	ROS_ERROR("pRRT: There are no valid initial states!");
	return false;	
    }    

    ROS_INFO("pRRT: Starting with %u states", m_nn.size());
    
    SolutionInfo sol;
    sol.solution = NULL;
    sol.approxsol = NULL;
    sol.approxdif = INFINITY;
    
    std::vector<boost::thread*> th(m_threadCount);
    for (unsigned int i = 0 ; i < m_threadCount ; ++i)
	th[i] = new boost::thread(boost::bind(&pRRT::threadSolve, this, i, endTime, &sol));
    for (unsigned int i = 0 ; i < m_threadCount ; ++i)
    {
	th[i]->join();
	delete th[i];
    }
    
    bool approximate = false;
    if (sol.solution == NULL)
    {
	sol.solution = sol.approxsol;
	approximate = true;
    }
    
    if (sol.solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion*> mpath;
	while (sol.solution != NULL)
	{
	    mpath.push_back(sol.solution);
	    sol.solution = sol.solution->parent;
	}

	/* set the solution path */
	PathKinematic *path = new PathKinematic(m_si);
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{   
	    base::State *st = new base::State(dim);
	    si->copyState(st, mpath[i]->state);
	    path->states.push_back(st);
	}
	goal->setDifference(sol.approxdif);
	goal->setSolutionPath(path, approximate);

	if (approximate)
	    ROS_WARN("pRRT: Found approximate solution");
    }

    ROS_INFO("pRRT: Created %u states", m_nn.size());
    
    return goal->isAchieved();
}

void ompl::kinematic::pRRT::getStates(std::vector<const base::State*> &states) const
{
    std::vector<Motion*> motions;
    m_nn.list(motions);
    states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	states[i] = motions[i]->state;
}

void ompl::kinematic::pRRT::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);		
    m_threadCount = nthreads;
    m_sCoreArray.resize(m_threadCount);
}
