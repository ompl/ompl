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

#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/base/GoalSampleableRegion.h"

void ompl::geometric::RRTConnect::setup(void)
{
    Planner::setup();
    checkMotionLength(this, maxDistance_);
   
    sampler_ = si_->allocStateSampler();
    
    if (!tStart_)
	tStart_.reset(new NearestNeighborsSqrtApprox<Motion*>());
    if (!tGoal_)
	tGoal_.reset(new NearestNeighborsSqrtApprox<Motion*>());
    tStart_->setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, this, _1, _2));
    tGoal_->setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, this, _1, _2));
}

void ompl::geometric::RRTConnect::freeMemory(void)
{
    std::vector<Motion*> motions;
    tStart_->list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
	if (motions[i]->state)
	    si_->freeState(motions[i]->state);
	delete motions[i];
    }
    
    tGoal_->list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
	if (motions[i]->state)
	    si_->freeState(motions[i]->state);
	delete motions[i];
    }
}

void ompl::geometric::RRTConnect::clear(void)
{
    freeMemory();
    tStart_->clear();
    tGoal_->clear();
}

ompl::geometric::RRTConnect::GrowState ompl::geometric::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;
    
    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
	si_->getStateManifold()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
	dstate = tgi.xstate;
	reach = false;
    }

    if (si_->checkMotion(nmotion->state, dstate))
    {
	/* create a motion */
	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, dstate);
	motion->parent = nmotion;
	motion->root = nmotion->root;
	tgi.xmotion = motion;
	
	tree->add(motion);
	if (reach)
	    return REACHED;
	else
	    return ADVANCED;
    }
    else
	return TRAPPED;
}

bool ompl::geometric::RRTConnect::solve(double solveTime)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    
    if (!goal)
    {
	msg_.error("Unknown type of goal (or goal undefined)");
	return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);

    while (const base::State *st = pis_.nextStart())
    {
	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, st);
	motion->root = st;
	tStart_->add(motion);
    }
    
    if (tStart_->size() == 0)
    {
	msg_.error("Motion planning start tree could not be initialized!");
	return false;
    }

    if (goal->maxSampleCount() <= 0)
    {
	msg_.error("Insufficient states in sampleable goal region");
	return false;
    }
    
    msg_.inform("Starting with %d states", (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();
    
    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool   startTree    = true;

    while (time::now() < endTime)
    {
	TreeData &tree      = startTree ? tStart_ : tGoal_;
	startTree = !startTree;
	TreeData &otherTree = startTree ? tStart_ : tGoal_;
		
	if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
	{
	    const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(endTime) : pis_.nextGoal();
	    if (st)
	    {
		Motion* motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->root = motion->state;
		tGoal_->add(motion);
	    }
	    
	    if (tGoal_->size() == 0)
	    {
		msg_.error("Unable to sample any valid states for goal tree");
		break;
	    }
	}

	/* sample random state */
	sampler_->sample(rstate);
	
	GrowState gs = growTree(tree, tgi, rmotion);
	
	if (gs != TRAPPED)
	{
	    /* remember which motion was just added */
	    Motion *addedMotion = tgi.xmotion;
	    
	    /* attempt to connect trees */
	    
	    /* if reached, it means we used rstate directly, no need top copy again */
	    if (gs != REACHED)
		si_->copyState(rstate, tgi.xstate);

	    GrowState gsc = ADVANCED;
	    while (gsc == ADVANCED)
		gsc = growTree(otherTree, tgi, rmotion);

	    /* if we connected the trees in a valid way (start and goal pair is valid)*/
	    if (gsc == REACHED && goal->isStartGoalPairValid(startTree ? tgi.xmotion->root : addedMotion->root,
							     startTree ? addedMotion->root : tgi.xmotion->root))
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

		PathGeometric *path = new PathGeometric(si_);
		for (unsigned int i = 0 ; i < sol.size() ; ++i)
		    path->states.push_back(si_->cloneState(sol[i]->state));
		
		goal->setDifference(0.0);
		goal->setSolutionPath(base::PathPtr(path));
		break;
	    }
	}
    }
    
    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;
    
    msg_.inform("Created %u states (%u start + %u goal)", tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());
    
    return goal->isAchieved();
}

void ompl::geometric::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    data.si = si_;
    std::vector<Motion*> motions;
    tStart_->list(motions);
    data.states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[i] = motions[i]->state;
    tGoal_->list(motions);
    unsigned int s = data.states.size();
    data.states.resize(s + motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[s + i] = motions[i]->state;
}
