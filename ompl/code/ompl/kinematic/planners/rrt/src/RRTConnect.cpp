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

#include "ompl/kinematic/planners/rrt/RRTConnect.h"
#include "ompl/base/GoalState.h"

ompl::kinematic::RRTConnect::GrowState ompl::kinematic::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree.nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;
    
    /* find state to add */
    for (unsigned int i = 0 ; i < tgi.dim ; ++i)
    {
	double diff = rmotion->state->values[i] - nmotion->state->values[i];
	if (fabs(diff) < tgi.range[i])
	    tgi.xstate->values[i] = rmotion->state->values[i];
	else
	{
	    tgi.xstate->values[i] = nmotion->state->values[i] + diff * m_rho;
	    reach = false;
	}
    }
    
    if (static_cast<SpaceInformationKinematic*>(m_si)->checkMotion(nmotion->state, tgi.xstate))
    {
	/* create a motion */
	Motion *motion = new Motion(tgi.dim);
	static_cast<SpaceInformationKinematic*>(m_si)->copyState(motion->state, tgi.xstate);
	motion->parent = nmotion;
	tgi.xmotion = motion;
	
	tree.add(motion);
	if (reach)
	    return REACHED;
	else
	    return ADVANCED;	
    }
    else
	return TRAPPED;    
}

bool ompl::kinematic::RRTConnect::solve(double solveTime)
{
    SpaceInformationKinematic *si   = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::GoalState           *goal = dynamic_cast<base::GoalState*>(si->getGoal());
    unsigned int                dim = si->getStateDimension();
    
    if (!goal)
    {
	m_msg.error("RRTConnect: Unknown type of goal (or goal undefined)");
	return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);

    if (m_tStart.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, si->getStartState(i));
	    if (si->satisfiesBounds(motion->state) && si->isValid(motion->state))
		m_tStart.add(motion);
	    else
	    {
		m_msg.error("RRTConnect: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_tGoal.size() == 0)
    {	   
	Motion *motion = new Motion(dim);
	si->copyState(motion->state, goal->state);
	if (si->satisfiesBounds(motion->state) && si->isValid(motion->state))
	    m_tGoal.add(motion);
	else
	{
	    m_msg.error("RRTConnect: Goal state is invalid!");
	    delete motion;
	}
    }
    
    if (m_tStart.size() == 0 || m_tGoal.size() == 0)
    {
	m_msg.error("RRTConnect: Motion planning trees could not be initialized!");
	return false;
    }
    
    m_msg.inform("RRTConnect: Starting with %d states", (int)(m_tStart.size() + m_tGoal.size()));

    TreeGrowingInfo tgi;
    tgi.range.resize(dim);
    tgi.dim = dim;
    for (unsigned int i = 0 ; i < dim ; ++i)
	tgi.range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    tgi.xstate = new base::State(dim);
    
    Motion *rmotion   = new Motion(dim);
    base::State *rstate = rmotion->state;
    bool   startTree    = true;

    while (time::now() < endTime)
    {
	TreeData &tree      = startTree ? m_tStart : m_tGoal;
	startTree = !startTree;
	TreeData &otherTree = startTree ? m_tStart : m_tGoal;
	
	/* sample random state */
	m_sCore->sample(rstate);
	
	GrowState gs = growTree(tree, tgi, rmotion);
	
	if (gs != TRAPPED)
	{
	    /* remember which motion was just added */
	    Motion *addedMotion = tgi.xmotion;
	    
	    /* attempt to connect trees */
	    si->copyState(rstate, tgi.xstate);
	    GrowState gsc = ADVANCED;
	    while (gsc == ADVANCED)
		gsc = growTree(otherTree, tgi, rmotion);

	    /* if we connected the trees */
	    if (gsc == REACHED)
	    {
		/* construct the solution path */
		Motion *solution = tgi.xmotion;
		std::vector<Motion*> mpath1;
		while (solution != NULL)
		{
		    mpath1.push_back(solution);
		    solution = solution->parent;
		}
		
		solution = addedMotion;
		std::vector<Motion*> mpath2;
		while (solution != NULL)
		{
		    mpath2.push_back(solution);
		    solution = solution->parent;
		}
		
		if (!startTree)
		    mpath2.swap(mpath1);

		std::vector<Motion*> sol;
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
		    sol.push_back(mpath1[i]);
		sol.insert(sol.end(), mpath2.begin(), mpath2.end());

		PathKinematic *path = new PathKinematic(m_si);
		for (unsigned int i = 0 ; i < sol.size() ; ++i)
		{
		    base::State *st = new base::State(dim);
		    si->copyState(st, sol[i]->state);
		    path->states.push_back(st);
		}
		
		goal->setDifference(0.0);
		goal->setSolutionPath(path);
		break;
	    }
	}
    }
    
    delete tgi.xstate;
    delete rmotion;
	
    m_msg.inform("RRTConnect: Created %u states (%u start + %u goal)", m_tStart.size() + m_tGoal.size(), m_tStart.size(), m_tGoal.size());
    
    return goal->isAchieved();
}

void ompl::kinematic::RRTConnect::getStates(std::vector<const base::State*> &states) const
{
    std::vector<Motion*> motions;
    m_tStart.list(motions);
    states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	states[i] = motions[i]->state;
    m_tGoal.list(motions);
    unsigned int s = states.size();
    states.resize(s + motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	states[s + i] = motions[i]->state;
}
